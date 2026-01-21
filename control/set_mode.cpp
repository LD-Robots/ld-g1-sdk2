#include <iostream>
#include <string>
#include <cstring>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

const char* get_fsm_description(int fsm_id) {
  switch (fsm_id) {
    case 0:   return "ZeroTorque - motors free (CAUTION: robot will fall!)";
    case 1:   return "Damp - damping mode";
    case 2:   return "Squat - squat position";
    case 3:   return "Sit - sitting position";
    case 4:   return "StandUp - stand up";
    case 500: return "Start - normal walking mode";
    case 501: return "Advanced - walking + arm control enabled";
    case 801: return "Expert - low-level control mode";
    default:  return "Unknown";
  }
}

void print_help(const char* program_name) {
  std::cout << "Usage: " << program_name << " <network_interface> [options]\n\n";
  std::cout << "Control G1 robot FSM states and movement.\n\n";
  std::cout << "Arguments:\n";
  std::cout << "  <network_interface>    Network interface (e.g., eth0, lo)\n\n";
  std::cout << "Options:\n";
  std::cout << "  --help                 Show this help message\n";
  std::cout << "  --get_fsm_id           Get current FSM ID\n";
  std::cout << "  --fsm_id=<id>          Set FSM ID (see FSM IDs below)\n";
  std::cout << "  --damp                 Set Damp mode (FSM 1)\n";
  std::cout << "  --start                Start walking mode (FSM 500)\n";
  std::cout << "  --squat                Squat position (FSM 2)\n";
  std::cout << "  --sit                  Sit position (FSM 3)\n";
  std::cout << "  --stand_up             Stand up (FSM 4)\n";
  std::cout << "  --zero_torque          Zero torque - motors free (FSM 0)\n";
  std::cout << "  --stop_move            Stop movement\n";
  std::cout << "  --high_stand           High stand position\n";
  std::cout << "  --low_stand            Low stand position\n";
  std::cout << "  --balance_stand        Balance stand mode\n";
  std::cout << "  --wave_hand            Wave hand gesture\n";
  std::cout << "  --shake_hand           Shake hand gesture\n";
  std::cout << "  --velocity=<vx,vy,w>   Set velocity (e.g., --velocity=0.3,0,0)\n";
  std::cout << "  --stand_height=<h>     Set stand height\n";
  std::cout << "  --swing_height=<h>     Set swing height\n";
  std::cout << "\nFSM IDs:\n";
  std::cout << "  0   - " << get_fsm_description(0) << "\n";
  std::cout << "  1   - " << get_fsm_description(1) << "\n";
  std::cout << "  2   - " << get_fsm_description(2) << "\n";
  std::cout << "  3   - " << get_fsm_description(3) << "\n";
  std::cout << "  4   - " << get_fsm_description(4) << "\n";
  std::cout << "  500 - " << get_fsm_description(500) << "\n";
  std::cout << "  501 - " << get_fsm_description(501) << "\n";
  std::cout << "  801 - " << get_fsm_description(801) << "\n";
  std::cout << "\nExamples:\n";
  std::cout << "  " << program_name << " eth0 --start\n";
  std::cout << "  " << program_name << " eth0 --fsm_id=501\n";
  std::cout << "  " << program_name << " eth0 --velocity=0.3,0,0\n";
  std::cout << "  " << program_name << " eth0 --stop_move --damp\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    print_help(argv[0]);
    return 1;
  }

  // Check for --help first
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      print_help(argv[0]);
      return 0;
    }
  }

  if (argc < 3) {
    std::cerr << "Error: Missing command option.\n";
    std::cerr << "Use --help for usage information.\n";
    return 1;
  }

  const std::string network_interface = argv[1];

  // Initialize channel factory
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  // Create and initialize client
  unitree::robot::g1::LocoClient client;
  client.Init();
  client.SetTimeout(10.f);

  int32_t ret = 0;
  bool command_executed = false;

  // Process all command line arguments
  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--get_fsm_id") {
      int fsm_id;
      ret = client.GetFsmId(fsm_id);
      std::cout << "Current FSM ID: " << fsm_id << " (" << get_fsm_description(fsm_id) << ") ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg.rfind("--fsm_id=", 0) == 0) {
      int fsm_id = std::stoi(arg.substr(9));
      ret = client.SetFsmId(fsm_id);
      std::cout << "SetFsmId(" << fsm_id << ") -> " << get_fsm_description(fsm_id) << " (ret: " << ret << ")\n";
      command_executed = true;
    }
    else if (arg == "--damp") {
      ret = client.Damp();
      std::cout << "Damp ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--start") {
      ret = client.Start();
      std::cout << "Start ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--squat") {
      ret = client.Squat();
      std::cout << "Squat ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--sit") {
      ret = client.Sit();
      std::cout << "Sit ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--stand_up") {
      ret = client.StandUp();
      std::cout << "StandUp ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--zero_torque") {
      ret = client.ZeroTorque();
      std::cout << "ZeroTorque ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--stop_move") {
      ret = client.StopMove();
      std::cout << "StopMove ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--high_stand") {
      ret = client.HighStand();
      std::cout << "HighStand ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--low_stand") {
      ret = client.LowStand();
      std::cout << "LowStand ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--balance_stand") {
      ret = client.BalanceStand();
      std::cout << "BalanceStand ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--wave_hand") {
      ret = client.WaveHand();
      std::cout << "WaveHand ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg == "--shake_hand") {
      ret = client.ShakeHand();
      std::cout << "ShakeHand ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg.rfind("--velocity=", 0) == 0) {
      std::string params = arg.substr(11);
      float vx = 0, vy = 0, omega = 0;
      if (sscanf(params.c_str(), "%f,%f,%f", &vx, &vy, &omega) >= 1) {
        ret = client.SetVelocity(vx, vy, omega);
        std::cout << "SetVelocity(" << vx << ", " << vy << ", " << omega << ") ret: " << ret << "\n";
        command_executed = true;
      } else {
        std::cerr << "Error: Invalid velocity format. Use --velocity=vx,vy,omega\n";
        return 1;
      }
    }
    else if (arg.rfind("--stand_height=", 0) == 0) {
      float height = std::stof(arg.substr(15));
      ret = client.SetStandHeight(height);
      std::cout << "SetStandHeight(" << height << ") ret: " << ret << "\n";
      command_executed = true;
    }
    else if (arg.rfind("--swing_height=", 0) == 0) {
      float height = std::stof(arg.substr(15));
      ret = client.SetSwingHeight(height);
      std::cout << "SetSwingHeight(" << height << ") ret: " << ret << "\n";
      command_executed = true;
    }
    else {
      std::cerr << "Error: Unknown option '" << arg << "'\n";
      std::cerr << "Use --help for usage information.\n";
      return 1;
    }
  }

  if (!command_executed) {
    std::cerr << "Error: No command executed.\n";
    return 1;
  }

  return 0;
}
