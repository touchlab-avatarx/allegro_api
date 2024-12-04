// Copyright (c) 2024 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <chrono>
#include <thread>

#include <allegro_api/allegro_api.hpp>
#include "allegro_api_implementation.hpp"

using namespace std::chrono_literals;

namespace allegro
{

AllegroHand::AllegroHand()
{
  impl_.reset(new Implementation());
}

AllegroHand::~AllegroHand()
{
  impl_.reset();
}

void AllegroHand::init(const std::string& port, int id)
{
  impl_->init(port, id);
}

bool AllegroHand::get_state(std::vector<double>& measured_position,
                 std::vector<double>& measured_velocity,
                 std::chrono::duration<long double> timeout)
{
  return impl_->get_state(measured_position, measured_velocity, timeout);
}

void AllegroHand::set_torque(const std::vector<double>& torque)
{
  impl_->set_torque(torque);
}


std::vector<std::string> AllegroHand::get_joint_names()
{
  return impl_->get_joint_names();
}

double AllegroHand::get_torque_limit(const std::string& joint_name)
{
  return impl_->get_torque_limit(joint_name);
}

void AllegroHand::set_torque_limit(const std::string& joint_name, double limit)
{
  impl_->set_torque_limit(joint_name, limit);
}

void AllegroHand::get_imu(std::vector<double>& quaternion)
{
  impl_->get_imu(quaternion);
}

std::string AllegroHand::get_serial()
{
  return impl_->serial_;
}

std::string AllegroHand::get_hand_version()
{
  return impl_->hand_version_;
}

std::string AllegroHand::get_firmware_version()
{
  return impl_->firmware_version_;
}

std::string AllegroHand::get_hand_type()
{
  return impl_->hand_type_;
}

double AllegroHand::get_finger_temperature(int id)
{
  if (id < 0 || id >= NUM_OF_FINGERS)
    return -1.0;
  return impl_->temperature_[id];
}

double AllegroHand::get_hand_temperature()
{
  impl_->can_request_information();
  std::this_thread::sleep_for(100us);
  return impl_->hand_temperature_;
}

void AllegroHand::set_device_id(uint8_t id)
{
  impl_->can_set_device_id(id);
  impl_->device_id_ = id;
}

}  // namespace allegro
