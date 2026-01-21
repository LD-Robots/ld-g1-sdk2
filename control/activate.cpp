#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <network_interface>\n";
    return 1;
  }

  const std::string network_interface = argv[1];
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  unitree::robot::g1::LocoClient client;
  client.Init();
  client.SetTimeout(10.f);

  int32_t ret = 0;
  ret = client.Damp();
  std::cout << "Damp ret: " << ret << "\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  ret = client.StandUp();
  std::cout << "StandUp ret: " << ret << "\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  ret = client.SetFsmId(501);
  std::cout << "SetFsmId(501) ret: " << ret << "\n";

  return 0;
}
