#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {
class AudioMicClient : public unitree::robot::g1::AudioClient {
 public:
  int32_t AsrOnce(std::string& out_text) {
    std::string parameter;
    std::string data;
    int32_t ret =
        Call(unitree::robot::g1::ROBOT_API_ID_AUDIO_ASR, parameter, data);
    out_text = data;
    return ret;
  }
};
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_audio_mic_test [NetWorkInterface(eth0)]"
              << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  AudioMicClient client;
  client.Init();
  client.SetTimeout(10.0f);

  std::cout << "Press Enter, then speak to the robot microphone." << std::endl;
  std::string line;
  std::getline(std::cin, line);

  std::string asr_result;
  int32_t ret = client.AsrOnce(asr_result);
  std::cout << "ASR API ret: " << ret << std::endl;
  std::cout << "ASR raw data: " << asr_result << std::endl;

  return ret == 0 ? 0 : 1;
}
