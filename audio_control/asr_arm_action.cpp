#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
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
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>

namespace {
constexpr const char* kAsrTopic = "rt/audio_msg";
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

unitree::robot::g1::G1ArmActionClient* g_client = nullptr;
std::atomic<uint64_t> g_last_asr_ms(0);
std::atomic<uint64_t> g_last_asr_msg_ms(0);
std::atomic<uint64_t> g_last_mic_ms(0);
std::atomic<bool> g_asr_seen(false);
std::mutex g_asr_mutex;
std::string g_pending_text;

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

bool ExtractJsonBool(const std::string& input,
                     const std::string& key,
                     bool* value) {
  const std::string pattern = "\"" + key + "\":";
  size_t pos = input.find(pattern);
  if (pos == std::string::npos) {
    return false;
  }
  pos += pattern.size();
  if (input.compare(pos, 4, "true") == 0) {
    *value = true;
    return true;
  }
  if (input.compare(pos, 5, "false") == 0) {
    *value = false;
    return true;
  }
  return false;
}

bool ExtractJsonString(const std::string& input,
                       const std::string& key,
                       std::string* value) {
  const std::string pattern = "\"" + key + "\":\"";
  size_t pos = input.find(pattern);
  if (pos == std::string::npos) {
    return false;
  }
  pos += pattern.size();
  size_t end = input.find('"', pos);
  if (end == std::string::npos || end <= pos) {
    return false;
  }
  *value = input.substr(pos, end - pos);
  return true;
}

void ProcessCommandText(const std::string& text) {
  if (g_client == nullptr) {
    return;
  }
  std::string normalized = Normalize(text);
  normalized = TrimPunctuation(normalized);
  if (normalized == "give me a hug" || normalized == "give me a hug please") {
    int32_t ret = g_client->ExecuteAction("hug");
    std::cout << "ASR command: \"hug\" ret=" << ret << std::endl;
    return;
  }
  if (normalized.rfind(kPrefix, 0) != 0) {
    return;
  }
  std::string action_name = normalized.substr(std::strlen(kPrefix));
  action_name = TrimPunctuation(action_name);
  if (action_name.empty()) {
    std::cout << "ASR command missing action name." << std::endl;
    return;
  }

  int32_t ret = 0;
  auto it = g_client->action_map.find(action_name);
  if (it != g_client->action_map.end()) {
    ret = g_client->ExecuteAction(it->second);
  } else {
    ret = g_client->ExecuteAction(action_name);
  }
  std::cout << "ASR command: \"" << action_name << "\" ret=" << ret
            << std::endl;
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
  bool wrote_wav = false;

  char buffer[kMicWavLenOnce];
  int total_bytes = 0;
  uint64_t last_no_data_ms = 0;
  while (true) {
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (len > 0) {
      g_last_mic_ms.store(unitree::common::GetCurrentTimeMillisecond());
      if (!wrote_wav) {
        size_t sample_count = static_cast<size_t>(len) / sizeof(int16_t);
        const int16_t* samples =
            reinterpret_cast<const int16_t*>(buffer);
        pcm_data.insert(pcm_data.end(), samples, samples + sample_count);
        total_bytes += static_cast<int>(len);
        if (total_bytes >= kMicWavLen) {
          WriteWav("record.wav", pcm_data);
          std::cout << "Mic listen: saved record.wav (" << kMicRecordSeconds
                    << "s)" << std::endl;
          wrote_wav = true;
          pcm_data.clear();
          pcm_data.shrink_to_fit();
        }
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

void AsrHandler(const void* msg) {
  if (g_client == nullptr) {
    return;
  }
  if (!g_asr_seen.exchange(true)) {
    std::cout << "ASR stream active." << std::endl;
  }
  g_last_asr_ms.store(unitree::common::GetCurrentTimeMillisecond());
  auto* res_msg = (std_msgs::msg::dds_::String_*)msg;
  std::string text = res_msg->data();
  std::cout << "ASR raw: " << text << std::endl;
  bool is_final = false;
  std::string asr_text;
  if (!ExtractJsonString(text, "text", &asr_text)) {
    return;
  }
  ExtractJsonBool(text, "is_final", &is_final);
  g_last_asr_msg_ms.store(unitree::common::GetCurrentTimeMillisecond());

  if (is_final) {
    ProcessCommandText(asr_text);
    return;
  }

  std::lock_guard<std::mutex> lock(g_asr_mutex);
  g_pending_text = asr_text;
}
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_asr_arm_action [NetWorkInterface(eth0)]"
              << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::G1ArmActionClient client;
  client.Init();
  client.SetTimeout(10.0f);
  g_client = &client;

  std::cout << "Listening for ASR commands on \"" << kAsrTopic << "\".\n"
            << "Say: execute <action_name>" << std::endl;

  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      kAsrTopic);
  subscriber.InitChannel(AsrHandler);

  std::thread mic_thread(MicListenThread, std::string(argv[1]));
  mic_thread.detach();

  while (true) {
    sleep(1);
    uint64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
    uint64_t last_msg = g_last_asr_msg_ms.load();
    uint64_t last_asr = g_last_asr_ms.load();
    uint64_t last_mic = g_last_mic_ms.load();
    if (last_asr == 0 || now_ms - last_asr > 5000) {
      std::cout << "Waiting for ASR data on \"" << kAsrTopic << "\"..."
                << std::endl;
      g_last_asr_ms.store(now_ms);
    }
    if (last_mic == 0 || now_ms - last_mic > 5000) {
      std::cout << "Waiting for mic stream on " << kMicGroupIp << ":"
                << kMicPort << "..." << std::endl;
      g_last_mic_ms.store(now_ms);
    }

    if (last_msg != 0 && now_ms - last_msg > 1500) {
      std::string pending;
      {
        std::lock_guard<std::mutex> lock(g_asr_mutex);
        pending.swap(g_pending_text);
      }
      if (!pending.empty()) {
        std::cout << "ASR timeout: treating partial as final." << std::endl;
        ProcessCommandText(pending);
      }
      g_last_asr_msg_ms.store(0);
    }
  }

  return 0;
}
