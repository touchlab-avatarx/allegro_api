// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <signal.h>
#include <string.h>

#include <string>
#include <iostream>
#include <sstream>
#include <thread>

#include <allegro_api/allegro_api.hpp>


using allegro::AllegroHand;

using namespace std::chrono_literals;

bool is_running;

void handler(int signal)
{
    is_running = false;
}

int main(int argc, char ** argv)
{
    std::string port(argv[argc - 1]);
    std::cout << "Port: " << port << "\n";
    AllegroHand hand;
    std::cout << "Initialising\n";
    hand.init(port);
    std::cout << "Hand version: " << hand.get_hand_version() << "\n";
    std::cout << "Hand type: " << hand.get_hand_type() << "\n";
    std::cout << "Firmware: " << hand.get_firmware_version() << "\n";
    std::cout << "Serial number: " << hand.get_serial() << "\n";
    std::cout << "Temperature: " << hand.get_hand_temperature() << "\n";
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> torque;
    position.assign(16, 0.0);
    velocity.assign(16, 0.0);
    torque.assign(16, 0.0);

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = handler;
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGINT, &action, NULL);

    is_running = true;
    while(is_running)
    {
        hand.get_state(position, velocity, -1s);
        hand.set_torque(torque);
        std::cout << position[0] << "\n";
        std::this_thread::yield();
    }
    return 0;
}
