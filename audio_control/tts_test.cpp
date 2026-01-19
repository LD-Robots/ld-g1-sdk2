#include <iostream>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_audio_tts_test [NetWorkInterface(eth0)]" << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  uint8_t volume = 0;
  int32_t vol_ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret: " << vol_ret
            << " volume: " << static_cast<int>(volume) << std::endl;

  int32_t ret = client.TtsMaker(
      "Hello. This is a G1 audio control TTS test in English.", 1);
  std::cout << "TtsMaker API ret: " << ret << std::endl;

  unitree::common::Sleep(2);
  return ret == 0 ? 0 : 1;
}
