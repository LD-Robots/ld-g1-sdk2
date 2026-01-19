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

  int R = 0, G = 0, B = 0;

  for (;B<255;B+=2) {
      std::cout << "LedControl Fade In Blue[" << R << "," << G << "," << B << "] " << ret << std::endl;
      ret = client.LedControl(R, G, B);
      unitree::common::MilliSleep(50);
  }
  B = 0;
  for (;G<255;G+=2) {
      std::cout << "LedControl Fade In Green[" << R << "," << G << "," << B << "] " << ret << std::endl;
      ret = client.LedControl(R, G, B);
      unitree::common::MilliSleep(50);
  }
  G = 0;
  for (;R<255;R+=2) {
      std::cout << "LedControl Fade In Red[" << R << "," << G << "," << B << "] " << ret << std::endl;
      ret = client.LedControl(R, G, B);
      unitree::common::MilliSleep(50);
  }

  ret = client.LedControl(0, 0, 0);
  return ret == 0 ? 0 : 1;
}
