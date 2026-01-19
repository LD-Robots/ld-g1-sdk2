#include <iostream>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_audio_led_test [NetWorkInterface(eth0)]"
              << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  std::cout << "LED test: red -> green -> blue -> white -> off" << std::endl;

  int32_t ret = client.LedControl(255, 0, 0);
  std::cout << "LedControl red ret: " << ret << std::endl;
  unitree::common::Sleep(1);

  ret = client.LedControl(0, 255, 0);
  std::cout << "LedControl green ret: " << ret << std::endl;
  unitree::common::Sleep(1);

  ret = client.LedControl(0, 0, 255);
  std::cout << "LedControl blue ret: " << ret << std::endl;
  unitree::common::Sleep(1);

  ret = client.LedControl(255, 255, 255);
  std::cout << "LedControl white ret: " << ret << std::endl;
  unitree::common::Sleep(1);

  ret = client.LedControl(0, 0, 0);
  std::cout << "LedControl off ret: " << ret << std::endl;

  return ret == 0 ? 0 : 1;
}
