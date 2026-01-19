#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/idl/hg/SportModeState_.hpp>
#include <whisper.h>

namespace {
constexpr const char* kPrefix = "execute ";
constexpr const char* kMicGroupIp = "239.168.123.161";
constexpr int kMicPort = 5555;
constexpr int kMicSampleRate = 16000;
constexpr int kMicChannels = 1;
constexpr int kMicBitsPerSample = 16;
constexpr int kMicRecordSeconds = 5;
constexpr int kMicBytesPerSample = kMicBitsPerSample / 8;
constexpr int kMicWavLen =
    kMicSampleRate * kMicChannels * kMicBytesPerSample * kMicRecordSeconds;
constexpr int kMicWavLenOnce =
    kMicSampleRate * kMicChannels * kMicBytesPerSample * 160 / 1000;
#ifndef WHISPER_MODEL_PATH
#define WHISPER_MODEL_PATH "thirdparty/whisper.cpp/models/ggml-base.en.bin"
#endif
constexpr const char* kDefaultModelPath = WHISPER_MODEL_PATH;
constexpr const char* kLocalMicTestPcm = "/tmp/whisper_mic_test.pcm";

unitree::robot::g1::G1ArmActionClient* g_client = nullptr;
unitree::robot::g1::AudioClient* g_audio_client = nullptr;
std::atomic<uint64_t> g_last_mic_ms(0);
std::atomic<uint64_t> g_last_whisper_ms(0);
std::atomic<uint32_t> g_fsm_id(0);
std::atomic<uint32_t> g_fsm_mode(0);
whisper_context* g_whisper_ctx = nullptr;

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

void ProcessCommandText(const std::string& text) {
  if (g_client == nullptr) {
    return;
  }
  std::string normalized = Normalize(text);
  normalized = TrimPunctuation(normalized);
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
    std::cout << "Command: \"throw_money\" ret=" << ret << std::endl;
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

void WriteWav(const std::string& path, const std::vector<int16_t>& pcm_data) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cout << "Failed to open " << path << " for writing." << std::endl;
    return;
  }

  uint32_t data_size = static_cast<uint32_t>(pcm_data.size() * sizeof(int16_t));
  uint32_t riff_size = 36 + data_size;
  uint16_t audio_format = 1;
  uint16_t num_channels = kMicChannels;
  uint32_t sample_rate = kMicSampleRate;
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

std::vector<int16_t> RecordLocalMicPcm(int seconds) {
  std::cout << "Local mic: using arecord default device." << std::endl;
  std::string cmd = std::string("arecord -q -f S16_LE -r 16000 -c 1 -d ") +
                    std::to_string(seconds) + " -t raw " + kLocalMicTestPcm;
  std::cout << "Local mic: command: " << cmd << std::endl;
  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    std::cout << "arecord failed, ret=" << ret << std::endl;
    return {};
  }
  std::vector<int16_t> data = ReadRawPcm(kLocalMicTestPcm);
  std::remove(kLocalMicTestPcm);
  return data;
}

std::string GetInterfaceIpv4(const std::string& iface) {
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    return "";
  }

  std::string result;
  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if (iface != ifa->ifa_name) {
      continue;
    }

    char host[NI_MAXHOST] = {0};
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST,
                    nullptr, 0, NI_NUMERICHOST) != 0) {
      continue;
    }

    result = host;
    break;
  }

  freeifaddrs(ifaddr);
  return result;
}

void MicListenThread(const std::string& iface) {
  std::cout << "Mic listen: thread started on interface " << iface << "."
            << std::endl;
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cout << "Mic listen: failed to create UDP socket." << std::endl;
    return;
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(kMicPort);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr),
           sizeof(local_addr)) < 0) {
    std::cout << "Mic listen: failed to bind UDP socket." << std::endl;
    close(sock);
    return;
  }

  ip_mreqn mreq{};
  if (inet_pton(AF_INET, kMicGroupIp, &mreq.imr_multiaddr) != 1) {
    std::cout << "Mic listen: failed to parse multicast IP." << std::endl;
    close(sock);
    return;
  }

  std::string local_ip = GetInterfaceIpv4(iface);
  std::cout << "Mic listen local ip: " << local_ip << std::endl;
  if (local_ip.empty()) {
    std::cout << "Mic listen: no IPv4 for interface " << iface << "."
              << std::endl;
    close(sock);
    return;
  }

  mreq.imr_address.s_addr = inet_addr(local_ip.c_str());
  mreq.imr_ifindex = if_nametoindex(iface.c_str());
  if (mreq.imr_ifindex == 0) {
    std::cout << "Mic listen: failed to resolve interface index for " << iface
              << "." << std::endl;
    close(sock);
    return;
  }

  if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &mreq,
                 sizeof(mreq)) < 0) {
    std::cout << "Mic listen: failed to set multicast interface." << std::endl;
  }
  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) < 0) {
    std::cout << "Mic listen: failed to join multicast group." << std::endl;
    close(sock);
    return;
  }

  timeval timeout{};
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <
      0) {
    std::cout << "Mic listen: failed to set socket timeout." << std::endl;
  }

  const size_t target_samples =
      static_cast<size_t>(kMicWavLen / sizeof(int16_t));
  std::vector<int16_t> pcm_data;
  pcm_data.reserve(target_samples);

  char buffer[kMicWavLenOnce];
  int total_bytes = 0;
  uint64_t last_no_data_ms = 0;
  while (true) {
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (len > 0) {
      g_last_mic_ms.store(unitree::common::GetCurrentTimeMillisecond());
      size_t sample_count = static_cast<size_t>(len) / sizeof(int16_t);
      const int16_t* samples =
          reinterpret_cast<const int16_t*>(buffer);
      pcm_data.insert(pcm_data.end(), samples, samples + sample_count);
      total_bytes += static_cast<int>(len);
      if (total_bytes >= kMicWavLen) {
        WriteWav("record.wav", pcm_data);
        std::cout << "Mic listen: saved record.wav (" << kMicRecordSeconds
                  << "s)" << std::endl;
        std::string transcript = TranscribeWithWhisper(pcm_data);
        if (!transcript.empty()) {
          std::cout << "Whisper text: " << transcript << std::endl;
          ProcessCommandText(transcript);
          g_last_whisper_ms.store(
              unitree::common::GetCurrentTimeMillisecond());
        }
        pcm_data.clear();
        pcm_data.shrink_to_fit();
        pcm_data.reserve(target_samples);
        total_bytes = 0;
      }
    } else {
      uint64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
      if (last_no_data_ms == 0 || now_ms - last_no_data_ms > 5000) {
        std::cout << "Mic listen: no data yet (errno=" << errno << ")"
                  << std::endl;
        last_no_data_ms = now_ms;
      }
    }
  }
}

void SportModeHandler(const void* msg) {
  auto* state = (unitree_hg::msg::dds_::SportModeState_*)msg;
  uint32_t new_id = state->fsm_id();
  uint32_t new_mode = state->fsm_mode();
  uint32_t old_id = g_fsm_id.exchange(new_id);
  uint32_t old_mode = g_fsm_mode.exchange(new_mode);
  if (new_id != old_id || new_mode != old_mode) {
    std::cout << "FSM state: id=" << new_id << " mode=" << new_mode
              << std::endl;
  }
}
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_asr_arm_action [NetWorkInterface(eth0)|TEST] [model_path]"
              << std::endl;
    return 1;
  }

  std::string model_path = kDefaultModelPath;
  if (argc >= 3) {
    model_path = argv[2];
  }

  if (std::string(argv[1]) == "TEST") {
    g_whisper_ctx = whisper_init_from_file(model_path.c_str());
    if (g_whisper_ctx == nullptr) {
      std::cout << "Failed to load Whisper model: " << model_path << std::endl;
      return 1;
    }
    std::cout << "Whisper model loaded: " << model_path << std::endl;
    std::cout << "Local mic devices:\n"
              << RunCommand("arecord -l 2>&1") << std::endl;
    std::cout << "Recording local mic for " << kMicRecordSeconds
              << " seconds..." << std::endl;
    std::vector<int16_t> pcm_data = RecordLocalMicPcm(kMicRecordSeconds);
    if (pcm_data.empty()) {
      std::cout << "No local mic data captured." << std::endl;
      return 1;
    }
    std::string transcript = TranscribeWithWhisper(pcm_data);
    std::cout << "Whisper text: " << transcript << std::endl;
    return 0;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::G1ArmActionClient client;
  client.Init();
  client.SetTimeout(10.0f);
  g_client = &client;

  unitree::robot::g1::AudioClient audio_client;
  audio_client.Init();
  audio_client.SetTimeout(10.0f);
  g_audio_client = &audio_client;

  g_whisper_ctx = whisper_init_from_file(model_path.c_str());
  if (g_whisper_ctx == nullptr) {
    std::cout << "Failed to load Whisper model: " << model_path << std::endl;
    return 1;
  }
  std::cout << "Whisper model loaded: " << model_path << std::endl;
  std::cout << "Listening for Whisper commands. Say: execute <action_name>"
            << std::endl;

  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::SportModeState_>
      fsm_subscriber("rt/sportmodestate");
  fsm_subscriber.InitChannel(SportModeHandler);

  std::thread mic_thread(MicListenThread, std::string(argv[1]));
  mic_thread.detach();

  while (true) {
    sleep(1);
    uint64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
    uint64_t last_mic = g_last_mic_ms.load();
    if (last_mic == 0 || now_ms - last_mic > 5000) {
      std::cout << "Waiting for mic stream on " << kMicGroupIp << ":"
                << kMicPort << "..." << std::endl;
      g_last_mic_ms.store(now_ms);
    }
    uint64_t last_whisper = g_last_whisper_ms.load();
    if (last_whisper == 0 || now_ms - last_whisper > 5000) {
      std::cout << "Waiting for Whisper transcription..." << std::endl;
      g_last_whisper_ms.store(now_ms);
    }
  }

  return 0;
}
