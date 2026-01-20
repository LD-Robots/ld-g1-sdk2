#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <curl/curl.h>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <rnnoise.h>
#include <whisper.h>

namespace {

constexpr int kMicCaptureRate = 48000;
constexpr int kMicWhisperRate = 16000;
constexpr int kMicChannels = 1;
constexpr int kMicBitsPerSample = 16;
constexpr int kMicChunkSeconds = 1;
constexpr int kMicMaxRecordSeconds = 5;
constexpr int kMicSilenceStopMs = 500;
constexpr float kMicVadThresholdStart = 0.0f;
constexpr float kMicVadThresholdContinue = 0.0f;
constexpr int kMicRmsThreshold = 1200;
constexpr int kMaxContextMessages = 10;

#ifndef WHISPER_MODEL_PATH
#define WHISPER_MODEL_PATH "thirdparty/whisper.cpp/models/ggml-tiny.en.bin"
#endif
constexpr const char* kDefaultModelPath = WHISPER_MODEL_PATH;
constexpr const char* kLocalMicChunkPcm = "/tmp/conv_mic_chunk.pcm";
constexpr const char* kDefaultAlsaDevice = "default";

std::string g_alsa_device = kDefaultAlsaDevice;

unitree::robot::g1::AudioClient* g_audio_client = nullptr;
whisper_context* g_whisper_ctx = nullptr;
DenoiseState* g_rnnoise_state = nullptr;
std::mutex g_queue_mutex;
std::condition_variable g_queue_cv;
std::deque<std::vector<int16_t>> g_pcm_queue;
std::atomic<bool> g_capture_running(true);

struct ChatMessage {
  std::string role;
  std::string content;
};

std::vector<ChatMessage> g_conversation_history;
std::mutex g_history_mutex;

std::string g_openai_api_key;
std::string g_openai_model = "gpt-4o-mini";
std::string g_system_prompt =
    "You are a friendly robot assistant named G1. You are helpful, concise, "
    "and speak naturally. Keep responses brief (1-2 sentences) since they "
    "will be spoken aloud. Be conversational and engaging.";

std::string EscapeJson(const std::string& input) {
  std::string output;
  output.reserve(input.size() * 2);
  for (char ch : input) {
    switch (ch) {
      case '"':
        output += "\\\"";
        break;
      case '\\':
        output += "\\\\";
        break;
      case '\b':
        output += "\\b";
        break;
      case '\f':
        output += "\\f";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(ch));
          output += buf;
        } else {
          output += ch;
        }
        break;
    }
  }
  return output;
}

std::string BuildMessagesJson(const std::vector<ChatMessage>& messages) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < messages.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << "{\"role\":\"" << messages[i].role << "\",\"content\":\""
        << EscapeJson(messages[i].content) << "\"}";
  }
  oss << "]";
  return oss.str();
}

std::string ExtractContentFromResponse(const std::string& json_response) {
  const std::string marker = "\"content\":";
  size_t pos = json_response.find(marker);
  if (pos == std::string::npos) {
    return "";
  }

  pos += marker.size();
  while (pos < json_response.size() &&
         (json_response[pos] == ' ' || json_response[pos] == '\t' ||
          json_response[pos] == '\n' || json_response[pos] == '\r')) {
    ++pos;
  }

  if (pos >= json_response.size()) {
    return "";
  }

  if (json_response[pos] == 'n') {
    return "";
  }

  if (json_response[pos] != '"') {
    return "";
  }

  ++pos;
  std::string content;
  while (pos < json_response.size()) {
    char ch = json_response[pos];
    if (ch == '"') {
      break;
    }
    if (ch == '\\' && pos + 1 < json_response.size()) {
      char next = json_response[pos + 1];
      switch (next) {
        case '"':
          content += '"';
          break;
        case '\\':
          content += '\\';
          break;
        case 'n':
          content += '\n';
          break;
        case 'r':
          content += '\r';
          break;
        case 't':
          content += '\t';
          break;
        case 'b':
          content += '\b';
          break;
        case 'f':
          content += '\f';
          break;
        default:
          content += next;
          break;
      }
      pos += 2;
      continue;
    }
    content += ch;
    ++pos;
  }

  return content;
}

size_t CurlWriteCallback(void* contents, size_t size, size_t nmemb,
                         void* userp) {
  size_t total_size = size * nmemb;
  std::string* response = static_cast<std::string*>(userp);
  response->append(static_cast<char*>(contents), total_size);
  return total_size;
}

std::string CallOpenAI(const std::string& user_message) {
  if (g_openai_api_key.empty()) {
    return "Error: OpenAI API key not set.";
  }

  std::vector<ChatMessage> messages;
  messages.push_back({"system", g_system_prompt});

  {
    std::lock_guard<std::mutex> lock(g_history_mutex);
    for (const auto& msg : g_conversation_history) {
      messages.push_back(msg);
    }
  }
  messages.push_back({"user", user_message});

  std::string messages_json = BuildMessagesJson(messages);
  std::ostringstream body;
  body << "{\"model\":\"" << g_openai_model << "\",\"messages\":"
       << messages_json << ",\"max_tokens\":150,\"temperature\":0.7}";
  std::string request_body = body.str();

  CURL* curl = curl_easy_init();
  if (!curl) {
    return "Error: Failed to initialize curl.";
  }

  std::string response;
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  std::string auth_header = "Authorization: Bearer " + g_openai_api_key;
  headers = curl_slist_append(headers, auth_header.c_str());

  curl_easy_setopt(curl, CURLOPT_URL,
                   "https://api.openai.com/v1/chat/completions");
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

  CURLcode res = curl_easy_perform(curl);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    return std::string("Error: curl request failed: ") + curl_easy_strerror(res);
  }

  std::string content = ExtractContentFromResponse(response);
  if (content.empty()) {
    std::cout << "OpenAI raw response: " << response << std::endl;
    // Try to extract error message from API response
    size_t err_pos = response.find("\"message\":");
    if (err_pos != std::string::npos) {
      std::string err_content = ExtractContentFromResponse(
          response.substr(err_pos - 1));
      if (!err_content.empty()) {
        return "API error: " + err_content;
      }
    }
    return "API returned empty response.";
  }

  return content;
}

void AddToHistory(const std::string& role, const std::string& content) {
  std::lock_guard<std::mutex> lock(g_history_mutex);
  g_conversation_history.push_back({role, content});
  while (g_conversation_history.size() > kMaxContextMessages) {
    g_conversation_history.erase(g_conversation_history.begin());
  }
}

std::string Normalize(const std::string& input) {
  std::string out;
  out.reserve(input.size());
  bool last_space = true;
  for (unsigned char ch : input) {
    if (std::isspace(ch)) {
      if (!last_space) {
        out.push_back(' ');
        last_space = true;
      }
      continue;
    }
    out.push_back(static_cast<char>(std::tolower(ch)));
    last_space = false;
  }
  if (!out.empty() && out.back() == ' ') {
    out.pop_back();
  }
  return out;
}

std::string TranscribeWithWhisper(const std::vector<int16_t>& pcm_data) {
  if (g_whisper_ctx == nullptr || pcm_data.empty()) {
    return "";
  }

  std::vector<float> samples(pcm_data.size());
  for (size_t i = 0; i < pcm_data.size(); ++i) {
    samples[i] = static_cast<float>(pcm_data[i]) / 32768.0f;
  }

  whisper_full_params params =
      whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  params.print_progress = false;
  params.print_realtime = false;
  params.print_timestamps = false;
  params.translate = false;
  params.language = "en";

  int ret = whisper_full(g_whisper_ctx, params, samples.data(),
                         static_cast<int>(samples.size()));
  if (ret != 0) {
    std::cout << "Whisper transcribe error: " << ret << std::endl;
    return "";
  }

  std::string result;
  int segments = whisper_full_n_segments(g_whisper_ctx);
  for (int i = 0; i < segments; ++i) {
    const char* segment = whisper_full_get_segment_text(g_whisper_ctx, i);
    if (segment != nullptr) {
      result += segment;
    }
  }
  return result;
}

struct RnnoiseChunkResult {
  std::vector<int16_t> denoised;
  float avg_vad = 0.0f;
};

RnnoiseChunkResult DenoiseChunk48k(const std::vector<int16_t>& pcm_data) {
  RnnoiseChunkResult result;
  if (g_rnnoise_state == nullptr || pcm_data.empty()) {
    return result;
  }

  constexpr int kFrameSize = 480;
  result.denoised.reserve(pcm_data.size());

  size_t offset = 0;
  float vad_sum = 0.0f;
  int vad_frames = 0;
  while (offset < pcm_data.size()) {
    float in_frame[kFrameSize] = {0.0f};
    float out_frame[kFrameSize] = {0.0f};
    size_t remaining = pcm_data.size() - offset;
    size_t frame_count =
        remaining < static_cast<size_t>(kFrameSize) ? remaining : kFrameSize;
    for (size_t i = 0; i < frame_count; ++i) {
      in_frame[i] = static_cast<float>(pcm_data[offset + i]) / 32768.0f;
    }
    float vad = rnnoise_process_frame(g_rnnoise_state, out_frame, in_frame);
    vad_sum += vad;
    vad_frames++;
    for (size_t i = 0; i < frame_count; ++i) {
      float v = out_frame[i];
      if (v > 1.0f) {
        v = 1.0f;
      } else if (v < -1.0f) {
        v = -1.0f;
      }
      result.denoised.push_back(static_cast<int16_t>(v * 32767.0f));
    }
    offset += frame_count;
  }

  if (vad_frames > 0) {
    result.avg_vad = vad_sum / vad_frames;
  }
  return result;
}

std::vector<int16_t> DownsampleTo16k(const std::vector<int16_t>& pcm_data) {
  if (pcm_data.empty()) {
    return {};
  }
  std::vector<int16_t> out;
  out.reserve(pcm_data.size() / 3);
  for (size_t i = 0; i + 2 < pcm_data.size(); i += 3) {
    out.push_back(pcm_data[i]);
  }
  return out;
}

std::vector<int16_t> ReadRawPcm(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return {};
  }
  in.seekg(0, std::ios::end);
  std::streamsize size = in.tellg();
  in.seekg(0, std::ios::beg);
  if (size <= 0 || size % static_cast<std::streamsize>(sizeof(int16_t)) != 0) {
    return {};
  }
  std::vector<int16_t> data(static_cast<size_t>(size / sizeof(int16_t)));
  if (!in.read(reinterpret_cast<char*>(data.data()), size)) {
    return {};
  }
  return data;
}

int ComputeRms(const std::vector<int16_t>& pcm) {
  if (pcm.empty()) {
    return 0;
  }
  double sum_sq = 0.0;
  for (int16_t sample : pcm) {
    double v = static_cast<double>(sample);
    sum_sq += v * v;
  }
  double rms = std::sqrt(sum_sq / pcm.size());
  return static_cast<int>(rms);
}

std::vector<int16_t> RecordLocalMicPcmDynamic() {
  std::cout << "\n[Listening...] Speak now." << std::endl;
  std::cout.flush();
  std::vector<int16_t> result;
  bool started = false;
  int silence_ms = 0;
  int captured_ms = 0;

  while (captured_ms < kMicMaxRecordSeconds * 1000) {
    std::string cmd =
        std::string("arecord -q -D ") + g_alsa_device + " -f S16_LE -r " +
        std::to_string(kMicCaptureRate) + " -c 1 -d " +
        std::to_string(kMicChunkSeconds) + " -t raw " + kLocalMicChunkPcm;
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
      std::cout << "arecord failed, ret=" << ret << std::endl;
      break;
    }
    std::vector<int16_t> chunk = ReadRawPcm(kLocalMicChunkPcm);
    std::remove(kLocalMicChunkPcm);
    if (chunk.empty()) {
      break;
    }

    RnnoiseChunkResult denoised = DenoiseChunk48k(chunk);
    if (denoised.denoised.empty()) {
      break;
    }
    int rms = ComputeRms(denoised.denoised);
    if (!started) {
      if (denoised.avg_vad >= kMicVadThresholdStart &&
          rms >= kMicRmsThreshold) {
        started = true;
        std::cout << "[Speech detected]" << std::endl;
        result.insert(result.end(), denoised.denoised.begin(),
                      denoised.denoised.end());
      }
    } else {
      result.insert(result.end(), denoised.denoised.begin(),
                    denoised.denoised.end());
      if (denoised.avg_vad < kMicVadThresholdContinue ||
          rms < kMicRmsThreshold) {
        silence_ms += kMicChunkSeconds * 1000;
      } else {
        silence_ms = 0;
      }
      if (silence_ms >= kMicSilenceStopMs) {
        std::cout << "[End of speech]" << std::endl;
        break;
      }
    }
    captured_ms += kMicChunkSeconds * 1000;
  }

  if (!started) {
    return {};
  }

  return result;
}

void CaptureThread() {
  while (g_capture_running.load()) {
    std::vector<int16_t> pcm_data = RecordLocalMicPcmDynamic();
    if (pcm_data.empty()) {
      unitree::common::Sleep(1);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(g_queue_mutex);
      g_pcm_queue.push_back(std::move(pcm_data));
    }
    g_queue_cv.notify_one();
  }
}

void SpeakResponse(const std::string& text) {
  if (g_audio_client == nullptr) {
    std::cout << "[Would speak]: " << text << std::endl;
    return;
  }

  std::cout << "[Speaking]: " << text << std::endl;
  g_audio_client->TtsMaker(text, 1);
}

}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: conv_main [NetworkInterface(eth0)|TEST] [model_path]"
              << std::endl;
    std::cout << "Environment: OPENAI_API_KEY must be set" << std::endl;
    std::cout << "Optional: OPENAI_MODEL (default: gpt-4o-mini)" << std::endl;
    std::cout << "Optional: CONV_SYSTEM_PROMPT (custom system prompt)"
              << std::endl;
    std::cout << "Optional: ALSA_DEVICE (default: default)" << std::endl;
    return 1;
  }

  const char* api_key_env = std::getenv("OPENAI_API_KEY");
  if (api_key_env == nullptr || std::string(api_key_env).empty()) {
    std::cout << "Error: OPENAI_API_KEY environment variable not set."
              << std::endl;
    return 1;
  }
  g_openai_api_key = api_key_env;

  const char* model_env = std::getenv("OPENAI_MODEL");
  if (model_env != nullptr && std::string(model_env).length() > 0) {
    g_openai_model = model_env;
  }

  const char* prompt_env = std::getenv("CONV_SYSTEM_PROMPT");
  if (prompt_env != nullptr && std::string(prompt_env).length() > 0) {
    g_system_prompt = prompt_env;
  }

  const char* alsa_env = std::getenv("ALSA_DEVICE");
  if (alsa_env != nullptr && std::string(alsa_env).length() > 0) {
    g_alsa_device = alsa_env;
  }

  std::string model_path = kDefaultModelPath;
  if (argc >= 3) {
    model_path = argv[2];
  }

  curl_global_init(CURL_GLOBAL_DEFAULT);

  whisper_context_params wparams = whisper_context_default_params();
  wparams.use_gpu = false;
  wparams.flash_attn = false;
  g_whisper_ctx =
      whisper_init_from_file_with_params(model_path.c_str(), wparams);
  if (g_whisper_ctx == nullptr) {
    std::cout << "Failed to load Whisper model: " << model_path << std::endl;
    curl_global_cleanup();
    return 1;
  }
  std::cout << "Whisper model loaded: " << model_path << std::endl;

  g_rnnoise_state = rnnoise_create(nullptr);
  if (g_rnnoise_state == nullptr) {
    std::cout << "Failed to init RNNoise." << std::endl;
    whisper_free(g_whisper_ctx);
    curl_global_cleanup();
    return 1;
  }

  const bool is_test = (std::string(argv[1]) == "TEST");

  std::unique_ptr<unitree::robot::g1::AudioClient> audio_client;
  if (!is_test) {
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    audio_client = std::make_unique<unitree::robot::g1::AudioClient>();
    audio_client->Init();
    audio_client->SetTimeout(10.0f);
    g_audio_client = audio_client.get();
  }

  std::cout << "\n========================================" << std::endl;
  std::cout << "G1 Conversational Mode" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Model: " << g_openai_model << std::endl;
  std::cout << "Audio: " << g_alsa_device << std::endl;
  std::cout << "Mode: " << (is_test ? "TEST (no robot)" : "LIVE") << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;
  std::cout << "========================================\n" << std::endl;

  std::thread capture_thread(CaptureThread);
  capture_thread.detach();

  while (true) {
    std::vector<int16_t> pcm_data;
    {
      std::unique_lock<std::mutex> lock(g_queue_mutex);
      g_queue_cv.wait(lock, [] { return !g_pcm_queue.empty(); });
      pcm_data = std::move(g_pcm_queue.front());
      g_pcm_queue.pop_front();
    }

    std::vector<int16_t> whisper_pcm = DownsampleTo16k(pcm_data);
    if (whisper_pcm.empty()) {
      continue;
    }

    std::string transcript = TranscribeWithWhisper(whisper_pcm);
    if (transcript.empty()) {
      std::cout << "[No speech detected]" << std::endl;
      continue;
    }

    std::string normalized = Normalize(transcript);
    if (normalized.length() < 2) {
      std::cout << "[Speech too short, ignoring]" << std::endl;
      continue;
    }

    std::cout << "\n[You said]: " << transcript << std::endl;

    if (normalized == "goodbye" || normalized == "bye" ||
        normalized == "exit" || normalized == "quit") {
      SpeakResponse("Goodbye! It was nice talking with you.");
      break;
    }

    if (normalized == "clear history" || normalized == "reset conversation" ||
        normalized == "start over") {
      std::lock_guard<std::mutex> lock(g_history_mutex);
      g_conversation_history.clear();
      SpeakResponse("Conversation history cleared. Let's start fresh!");
      continue;
    }

    std::cout << "[Thinking...]" << std::endl;
    std::string ai_response = CallOpenAI(transcript);

    AddToHistory("user", transcript);
    AddToHistory("assistant", ai_response);

    std::cout << "[G1]: " << ai_response << std::endl;
    SpeakResponse(ai_response);
  }

  g_capture_running.store(false);
  rnnoise_destroy(g_rnnoise_state);
  whisper_free(g_whisper_ctx);
  curl_global_cleanup();

  std::cout << "Conversational mode ended." << std::endl;
  return 0;
}
