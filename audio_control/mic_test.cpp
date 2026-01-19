#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {
constexpr const char* kAudioSubscribeTopic = "rt/audio_msg";
constexpr const char* kGroupIp = "239.168.123.161";
constexpr int kPort = 5555;
constexpr int kWavSeconds = 5;
constexpr int kSampleRate = 16000;
constexpr int kBytesPerSample = 2;
constexpr int kChannels = 1;
constexpr int kWavLen = kSampleRate * kBytesPerSample * kWavSeconds;
constexpr int kWavLenOnce = kSampleRate * kBytesPerSample * 160 / 1000;

void AsrHandler(const void* msg) {
  auto* res_msg = (std_msgs::msg::dds_::String_*)msg;
  std::cout << "ASR topic \"" << kAudioSubscribeTopic << "\" recv: "
            << res_msg->data() << std::endl;
}

std::string GetLocalIpForMulticast() {
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == -1) {
    return "";
  }

  std::string result;
  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }

    char host[NI_MAXHOST] = {0};
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST,
                    nullptr, 0, NI_NUMERICHOST) != 0) {
      continue;
    }

    std::string ip(host);
    if (ip.rfind("192.168.123.", 0) == 0) {
      result = ip;
      break;
    }
  }

  freeifaddrs(ifaddr);
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
  uint16_t num_channels = kChannels;
  uint32_t sample_rate = kSampleRate;
  uint16_t bits_per_sample = 16;
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

void MicRecordThread() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cout << "Failed to create UDP socket." << std::endl;
    return;
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(kPort);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr),
           sizeof(local_addr)) < 0) {
    std::cout << "Failed to bind UDP socket." << std::endl;
    close(sock);
    return;
  }

  ip_mreq mreq{};
  if (inet_pton(AF_INET, kGroupIp, &mreq.imr_multiaddr) != 1) {
    std::cout << "Failed to parse multicast IP." << std::endl;
    close(sock);
    return;
  }

  std::string local_ip = GetLocalIpForMulticast();
  std::cout << "local ip: " << local_ip << std::endl;
  if (local_ip.empty()) {
    std::cout << "No local IP in 192.168.123.x found for multicast."
              << std::endl;
    close(sock);
    return;
  }

  mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) < 0) {
    std::cout << "Failed to join multicast group." << std::endl;
    close(sock);
    return;
  }

  timeval timeout{};
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <
      0) {
    std::cout << "Failed to set socket timeout." << std::endl;
  }

  int total_bytes = 0;
  std::vector<int16_t> pcm_data;
  pcm_data.reserve(kWavLen / 2);
  std::cout << "start record! max 10 seconds" << std::endl;
  int64_t start_ms = unitree::common::GetCurrentTimeMillisecond();
  while (total_bytes < kWavLen) {
    int64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
    if (now_ms - start_ms >= 10000) {
      std::cout << "record timeout after 10 seconds." << std::endl;
      break;
    }
    char buffer[kWavLenOnce];
    ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
    if (len > 0) {
      size_t sample_count = static_cast<size_t>(len) / 2;
      const int16_t* samples = reinterpret_cast<const int16_t*>(buffer);
      pcm_data.insert(pcm_data.end(), samples, samples + sample_count);
      total_bytes += static_cast<int>(len);
      std::cout << "recorded bytes: " << total_bytes << "/" << kWavLen
                << std::endl;
    } else {
      std::cout << "recording... no data yet" << std::endl;
    }
  }

  if (pcm_data.empty()) {
    std::cout << "record finish! no audio captured." << std::endl;
  } else {
    WriteWav("record.wav", pcm_data);
    std::cout << "record finish! save to record.wav" << std::endl;
  }
  close(sock);
}
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_audio_mic_test [NetWorkInterface(eth0)]"
              << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      kAudioSubscribeTopic);
  subscriber.InitChannel(AsrHandler);

  std::cout << "Press Enter, then speak to the robot microphone." << std::endl;
  std::string line;
  std::getline(std::cin, line);

  std::cout << "AudioClient ASR example running. Waiting for ASR messages..."
            << std::endl;

  std::thread mic_thread(MicRecordThread);

  mic_thread.join();
  std::cout << "Recording complete. Waiting briefly for ASR messages..."
            << std::endl;
  sleep(3);
  return 0;
}
