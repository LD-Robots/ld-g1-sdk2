#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <net/if.h>
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

std::vector<int16_t> RecordMicPcm(const std::string& iface) {
  std::vector<int16_t> pcm_data;
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cout << "Failed to create UDP socket." << std::endl;
    return pcm_data;
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(kPort);
  local_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr),
           sizeof(local_addr)) < 0) {
    std::cout << "Failed to bind UDP socket." << std::endl;
    close(sock);
    return pcm_data;
  }

  ip_mreqn mreq{};
  if (inet_pton(AF_INET, kGroupIp, &mreq.imr_multiaddr) != 1) {
    std::cout << "Failed to parse multicast IP." << std::endl;
    close(sock);
    return pcm_data;
  }

  std::string local_ip = GetInterfaceIpv4(iface);
  std::cout << "local ip: " << local_ip << std::endl;
  if (local_ip.empty()) {
    std::cout << "No IPv4 found for interface " << iface << "." << std::endl;
    close(sock);
    return pcm_data;
  }

  mreq.imr_address.s_addr = inet_addr(local_ip.c_str());
  mreq.imr_ifindex = if_nametoindex(iface.c_str());
  if (mreq.imr_ifindex == 0) {
    std::cout << "Failed to resolve interface index for " << iface << "."
              << std::endl;
    close(sock);
    return pcm_data;
  }

  if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &mreq,
                 sizeof(mreq)) < 0) {
    std::cout << "Failed to set multicast interface: errno=" << errno
              << std::endl;
  }
  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) < 0) {
    std::cout << "Failed to join multicast group: errno=" << errno
              << std::endl;
    close(sock);
    return pcm_data;
  }

  timeval timeout{};
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <
      0) {
    std::cout << "Failed to set socket timeout." << std::endl;
  }

  int total_bytes = 0;
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
      std::cout << "recording... no data yet (errno=" << errno << ")"
                << std::endl;
    }
  }

  close(sock);
  return pcm_data;
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

  uint8_t volume = 0;
  int32_t volume_ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret: " << volume_ret
            << " volume: " << static_cast<int>(volume) << std::endl;

  std::cout << "Press Enter, then speak to the robot microphone." << std::endl;
  std::string line;
  std::getline(std::cin, line);

  std::cout << "Test 1: receive microphone audio (no ASR)..." << std::endl;

  std::vector<int16_t> pcm_data = RecordMicPcm(argv[1]);
  if (pcm_data.empty()) {
    std::cout << "record finish! no audio captured." << std::endl;
  } else {
    WriteWav("record.wav", pcm_data);
    std::cout << "record finish! save to record.wav" << std::endl;
  }

  if (!pcm_data.empty()) {
    std::vector<uint8_t> pcm_bytes(
        reinterpret_cast<uint8_t*>(pcm_data.data()),
        reinterpret_cast<uint8_t*>(pcm_data.data()) +
            pcm_data.size() * sizeof(int16_t));
    std::string stream_id =
        std::to_string(unitree::common::GetCurrentTimeMillisecond());
    std::cout << "Test 1b: play recorded audio back via PlayStream..."
              << std::endl;
    int32_t play_ret = client.PlayStream("mic_test", stream_id, pcm_bytes);
    std::cout << "PlayStream API ret: " << play_ret << std::endl;
    client.PlayStop(stream_id);
  }

  std::cout << "Test 2: ASR messages (if available)..." << std::endl;
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      kAudioSubscribeTopic);
  subscriber.InitChannel(AsrHandler);
  sleep(5);
  return 0;
}
