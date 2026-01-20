#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <rnnoise.h>
#include <whisper.h>

namespace {
constexpr const char* kPrefix = "execute ";
constexpr int kMicCaptureRate = 48000;
constexpr int kMicWhisperRate = 16000;
constexpr int kMicChannels = 1;
constexpr int kMicBitsPerSample = 16;
constexpr int kMicChunkSeconds = 1;
constexpr int kMicMaxRecordSeconds = 3;
constexpr int kMicSilenceStopMs = 400;
constexpr int kMicStartRmsThreshold = 250;
constexpr float kMicVadThresholdStart = 0.0022f;
constexpr float kMicVadThresholdContinue = 0.0018f;
#ifndef WHISPER_MODEL_PATH
#define WHISPER_MODEL_PATH "thirdparty/whisper.cpp/models/ggml-base.en.bin"
#endif
constexpr const char* kDefaultModelPath = WHISPER_MODEL_PATH;
constexpr const char* kLocalMicChunkPcm = "/tmp/whisper_mic_chunk.pcm";

unitree::robot::g1::G1ArmActionClient* g_client = nullptr;
unitree::robot::g1::AudioClient* g_audio_client = nullptr;
whisper_context* g_whisper_ctx = nullptr;
DenoiseState* g_rnnoise_state = nullptr;

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

std::string TrimPunctuation(const std::string& input) {
  size_t end = input.size();
  while (end > 0) {
    char ch = input[end - 1];
    if (ch == '.' || ch == ',' || ch == '!' || ch == '?' || ch == ';' ||
        ch == ':') {
      --end;
    } else {
      break;
    }
  }
  return input.substr(0, end);
}

bool IsCommandCandidate(const std::string& normalized) {
  return normalized.find("execute ") != std::string::npos ||
         normalized.find("hug") != std::string::npos ||
         normalized.find("throw money") != std::string::npos ||
         normalized.find("scratch head") != std::string::npos ||
         normalized.find("stop") != std::string::npos ||
         normalized.find("i miss you") != std::string::npos;
}

void ProcessCommandText(const std::string& text) {
  if (g_client == nullptr) {
    return;
  }
  std::string normalized = Normalize(text);
  normalized = TrimPunctuation(normalized);

  if (normalized == "stop" || normalized == "stop action" ||
      normalized == "stop actions") {
    std::cout << "Command: \"stop\"" << std::endl;
    g_client->StopCustomAction();
    g_client->ExecuteAction(99);
    return;
  }

  if (normalized == "give me a hug" || normalized == "give me a hug please") {
    int32_t ret = g_client->ExecuteAction(19);
    std::cout << "Command: \"hug\" ret=" << ret << std::endl;
    return;
  }

  if (normalized.find("i miss you") != std::string::npos) {
    std::cout << "TTS: \"come here to give you a hug\"" << std::endl;
    if (g_audio_client != nullptr) {
      g_audio_client->TtsMaker("Come here to give you a hug.", 1);
    }
    unitree::common::Sleep(2);
    int32_t ret = g_client->ExecuteAction(19);
    std::cout << "Command: \"hug\" ret=" << ret << std::endl;
    return;
  }

  if (normalized.find("throw money") != std::string::npos ||
      normalized.find("throw the money") != std::string::npos ||
      normalized.find("trow money") != std::string::npos ||
      normalized.find("trow the money") != std::string::npos) {
    int32_t ret = g_client->ExecuteAction("Throw_money");
    std::cout << "Command: \"Throw_money\" ret=" << ret << std::endl;
    return;
  }

  if (normalized == "scratch head" || normalized == "scratch my head") {
    int32_t ret = g_client->ExecuteAction("scratch_head");
    std::cout << "Command: \"scratch_head\" ret=" << ret << std::endl;
    return;
  }

  if (normalized.rfind(kPrefix, 0) != 0) {
    return;
  }

  std::string action_name = normalized.substr(std::strlen(kPrefix));
  action_name = TrimPunctuation(action_name);
  if (action_name.empty()) {
    std::cout << "Command missing action name." << std::endl;
    return;
  }

  int32_t ret = 0;
  auto it = g_client->action_map.find(action_name);
  if (it != g_client->action_map.end()) {
    ret = g_client->ExecuteAction(it->second);
  } else {
    ret = g_client->ExecuteAction(action_name);
  }
  std::cout << "Command: \"" << action_name << "\" ret=" << ret
            << std::endl;
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

void WriteWav(const std::string& path,
              const std::vector<int16_t>& pcm_data,
              int sample_rate_hz) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cout << "Failed to open " << path << " for writing." << std::endl;
    return;
  }

  uint32_t data_size = static_cast<uint32_t>(pcm_data.size() * sizeof(int16_t));
  uint32_t riff_size = 36 + data_size;
  uint16_t audio_format = 1;
  uint16_t num_channels = kMicChannels;
  uint32_t sample_rate = static_cast<uint32_t>(sample_rate_hz);
  uint16_t bits_per_sample = kMicBitsPerSample;
  uint32_t byte_rate = sample_rate * num_channels * bits_per_sample / 8;
  uint16_t block_align = num_channels * bits_per_sample / 8;

  out.write("RIFF", 4);
  out.write(reinterpret_cast<char*>(&riff_size), sizeof(riff_size));
  out.write("WAVE", 4);
  out.write("fmt ", 4);
  uint32_t fmt_chunk_size = 16;
  out.write(reinterpret_cast<char*>(&fmt_chunk_size), sizeof(fmt_chunk_size));
  out.write(reinterpret_cast<char*>(&audio_format), sizeof(audio_format));
  out.write(reinterpret_cast<char*>(&num_channels), sizeof(num_channels));
  out.write(reinterpret_cast<char*>(&sample_rate), sizeof(sample_rate));
  out.write(reinterpret_cast<char*>(&byte_rate), sizeof(byte_rate));
  out.write(reinterpret_cast<char*>(&block_align), sizeof(block_align));
  out.write(reinterpret_cast<char*>(&bits_per_sample), sizeof(bits_per_sample));
  out.write("data", 4);
  out.write(reinterpret_cast<char*>(&data_size), sizeof(data_size));
  out.write(reinterpret_cast<const char*>(pcm_data.data()), data_size);
}

std::vector<int16_t> ReadRawPcm(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cout << "Failed to open PCM file: " << path << std::endl;
    return {};
  }
  in.seekg(0, std::ios::end);
  std::streamsize size = in.tellg();
  in.seekg(0, std::ios::beg);
  if (size <= 0 || size % static_cast<std::streamsize>(sizeof(int16_t)) != 0) {
    std::cout << "Invalid PCM size: " << size << std::endl;
    return {};
  }
  std::vector<int16_t> data(static_cast<size_t>(size / sizeof(int16_t)));
  if (!in.read(reinterpret_cast<char*>(data.data()), size)) {
    std::cout << "Failed to read PCM data." << std::endl;
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

std::string RunCommand(const std::string& cmd) {
  std::string output;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    return "Failed to run command: " + cmd + "\n";
  }
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }
  pclose(pipe);
  return output;
}

std::vector<int16_t> RecordLocalMicPcmDynamic() {
  std::cout << "Local mic: using arecord default device." << std::endl;
  std::cout.flush();
  std::vector<int16_t> result;
  bool started = false;
  int silence_ms = 0;
  int captured_ms = 0;

  while (captured_ms < kMicMaxRecordSeconds * 1000) {
    std::string cmd =
        std::string("arecord -q -f S16_LE -r ") +
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
    std::cout << "VAD=" << denoised.avg_vad << " RMS=" << rms << std::endl;
    if (!started) {
      if (denoised.avg_vad >= kMicVadThresholdStart) {
        started = true;
        std::cout << "Speech start detected." << std::endl;
        result.insert(result.end(), denoised.denoised.begin(),
                      denoised.denoised.end());
      }
    } else {
      result.insert(result.end(), denoised.denoised.begin(),
                    denoised.denoised.end());
      if (denoised.avg_vad < kMicVadThresholdContinue) {
        silence_ms += kMicChunkSeconds * 1000;
      } else {
        silence_ms = 0;
      }
      if (silence_ms >= kMicSilenceStopMs) {
        std::cout << "Speech end detected." << std::endl;
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
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout
        << "Usage: g1_asr_arm_action [NetWorkInterface(eth0)|TEST] [model_path]"
        << std::endl;
    return 1;
  }

  std::string model_path = kDefaultModelPath;
  if (argc >= 3) {
    model_path = argv[2];
  }

  whisper_context_params wparams = whisper_context_default_params();
  wparams.use_gpu = false;
  wparams.flash_attn = false;
  g_whisper_ctx = whisper_init_from_file_with_params(model_path.c_str(),
                                                     wparams);
  if (g_whisper_ctx == nullptr) {
    std::cout << "Failed to load Whisper model: " << model_path << std::endl;
    return 1;
  }
  std::cout << "Whisper model loaded: " << model_path << std::endl;

  g_rnnoise_state = rnnoise_create(nullptr);
  if (g_rnnoise_state == nullptr) {
    std::cout << "Failed to init RNNoise." << std::endl;
    return 1;
  }

  const bool is_test = (std::string(argv[1]) == "TEST");
  if (is_test) {
    std::cout << "Local mic devices:\n"
              << RunCommand("arecord -l 2>&1") << std::endl;
  } else {
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  }

  std::unique_ptr<unitree::robot::g1::G1ArmActionClient> client;
  std::unique_ptr<unitree::robot::g1::AudioClient> audio_client;
  if (!is_test) {
    client = std::make_unique<unitree::robot::g1::G1ArmActionClient>();
    client->Init();
    client->SetTimeout(10.0f);
    g_client = client.get();

    audio_client = std::make_unique<unitree::robot::g1::AudioClient>();
    audio_client->Init();
    audio_client->SetTimeout(10.0f);
    g_audio_client = audio_client.get();

    std::cout << "Listening for Whisper commands. Say: execute <action_name>"
              << std::endl;
  } else {
    std::cout << "Listening for Whisper transcription from local mic."
              << std::endl;
  }

  while (true) {
    std::cout << "Capture loop start." << std::endl;
    std::vector<int16_t> pcm_data = RecordLocalMicPcmDynamic();
    if (pcm_data.empty()) {
      unitree::common::Sleep(1);
      continue;
    }
    std::vector<int16_t> whisper_pcm = DownsampleTo16k(pcm_data);
    if (whisper_pcm.empty()) {
      continue;
    }
    WriteWav("record.wav", whisper_pcm, kMicWhisperRate);
    std::string transcript = TranscribeWithWhisper(whisper_pcm);
    if (transcript.empty()) {
      std::cout << "Whisper text: <empty>" << std::endl;
      continue;
    }
    std::string normalized = Normalize(transcript);
    normalized = TrimPunctuation(normalized);
    if (!IsCommandCandidate(normalized)) {
      std::cout << "Whisper text ignored: " << transcript << std::endl;
      continue;
    }
    std::cout << "Whisper text: " << transcript << std::endl;
    if (!is_test) {
      ProcessCommandText(transcript);
    }
  }
}
