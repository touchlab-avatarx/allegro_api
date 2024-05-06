// Copyright (c) 2024 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <signal.h>
#include <string.h>
#include <getopt.h>

#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <allegro_api/allegro_api.hpp>
#include "utils.hpp"

using allegro::AllegroHand;

using namespace std::chrono_literals;

bool is_running;

void handler(int signal)
{
  is_running = false;
}

enum ACTION
{
  ACTION_NONE,
  ACTION_LIST,
  ACTION_INFO,
  ACTION_SET_ID,
  ACTION_PRINT_POSITION,
  ACTION_PRINT_VELOCITY,
};

int main(int argc, char** argv)
{
  int opt;
  int num_options = 0;
  uint8_t new_device_id = 0;
  bool needs_id = true;
  ACTION action = ACTION_NONE;

  while ((opt = getopt(argc, argv, "lipvd:")) != -1)
  {
    num_options++;
    switch (opt)
    {
    case 'l':
      action = ACTION_LIST;
      needs_id = false;
      break;
    case 'i':
      action = ACTION_INFO;
      break;
    case 'd':
      new_device_id = std::stoul(std::string(optarg));
      if (new_device_id > 3)
      {
        INFO("New device ID must be in range 0..3");
        return 1;
      }
      action = ACTION_SET_ID;
      break;
    case 'p':
      action = ACTION_PRINT_POSITION;
      break;
    case 'v':
      action = ACTION_PRINT_VELOCITY;
      break;
    case '?':
      num_options = 0;
      opt = -1;
      break;
    }
  }


  std::vector<std::string> args;
  if (optind < argc)
  {
    while (optind < argc)
      args.push_back(std::string(argv[optind++]));
  }

  if (num_options == 0 || (args.size() < 1))
  {
    if (!needs_id && args.size() != 1) INFO("Missing can interface");
    INFO("Usage:\n" <<
      "allegro_api [-l] <can_interface>\n" <<
      "allegro_api [-ipv] [-d new_device_id] <can_interface> [<device_id>]");
    INFO("Options:");
    INFO("  -l \t\tList devices");
    INFO("  -i \t\tPrint device info");
    INFO("  -p \t\tPrint joint positions");
    INFO("  -v \t\tPrint joint velocities");
    INFO("  -d id\t\tSet device id");
    return 1;
  }

  std::string port = args[0];
  uint8_t device_id = 0;
  if (args.size() >= 2)
  {
    device_id = std::stoul(std::string(args[1]));
    if (device_id > 3)
    {
      INFO("Device ID must be in range 0..3");
      return 1;
    }
  }

  try
  {
    switch (action)
    {
      case ACTION_INFO:
        {
          AllegroHand hand;
          hand.init(port, device_id);
          INFO("CAN port " << port);
          INFO("Device ID: " << (int)device_id);
          if (hand.get_hand_version() == "00")
          {
            INFO("Device not connected");
            return 1;
          }
          INFO("Allegro " << hand.get_hand_type() << " hand v" << hand.get_hand_version());
          INFO("Firmware v" << hand.get_firmware_version());
          INFO("Serial number: " << hand.get_serial());
          double temp = hand.get_hand_temperature();
          INFO("Hand temperature " << temp << "°C");
        }
        break;
      case ACTION_LIST:
        {
          int num_devices = 0;
          INFO("CAN port " << port);
          for (device_id = 0; device_id < 4; ++device_id)
          {
            AllegroHand hand;
            hand.init(port, device_id);
            std::string serial = hand.get_serial();
            if (serial != "")
            {
              num_devices++;
              INFO((int)device_id << " - Allegro " << hand.get_hand_type() << " hand " <<
                  hand.get_serial());
            }
          }
          if (num_devices == 0) INFO("No devices found");
        }
        break;
      case ACTION_SET_ID:
        INFO("Set device ID " << port << " " << (int)device_id << " to " << (int)new_device_id);
        {
          AllegroHand hand;
          hand.init(port, device_id);
          hand.set_device_id(new_device_id);
          INFO("Done. Reboot your device now.");
        }
        break;
      case ACTION_PRINT_POSITION:
      case ACTION_PRINT_VELOCITY:
        {
          is_running = true;
          struct sigaction signal_action;
          memset(&signal_action, 0, sizeof(struct sigaction));
          signal_action.sa_handler = handler;
          sigaction(SIGTERM, &signal_action, NULL);
          sigaction(SIGINT, &signal_action, NULL);

          AllegroHand hand;
          hand.init(port, device_id);

          std::vector<double> position;
          std::vector<double> velocity;
          position.assign(16, 0.0);
          velocity.assign(16, 0.0);

          while(is_running)
          {
              hand.get_state(position, velocity, -1s);
              if (action == ACTION_PRINT_VELOCITY) position = velocity;
              INFO("----------");
              std::cout.setf(std::ios::fixed, std::ios::floatfield);
              std::cout << std::setprecision(6);
              INFO(std::setw(10) << std::left << "Index" <<
                   std::setw(14) << std::right << position[0] <<
                   std::setw(14) << std::right << position[1] <<
                   std::setw(14) << std::right << position[2] <<
                   std::setw(14) << std::right << position[3]);
              INFO(std::setw(10) << std::left << "Middle" <<
                   std::setw(14) << std::right << position[4] <<
                   std::setw(14) << std::right << position[5] <<
                   std::setw(14) << std::right << position[6] <<
                   std::setw(14) << std::right << position[7]);
              INFO(std::setw(10) << std::left << "Pinky" <<
                   std::setw(14) << std::right << position[8] <<
                   std::setw(14) << std::right << position[9] <<
                   std::setw(14) << std::right << position[10] <<
                   std::setw(14) << std::right << position[11]);
              INFO(std::setw(10) << std::left << "Thumb" <<
                   std::setw(14) << std::right << position[12] <<
                   std::setw(14) << std::right << position[13] <<
                   std::setw(14) << std::right << position[14] <<
                   std::setw(14) << std::right << position[15]);
              std::this_thread::sleep_for(100ms);
          }
        }
        break;
    }
  }
  catch(const std::runtime_error& e)
  {
    INFO(e.what());
    return 1;
  }

  return 0;
}
