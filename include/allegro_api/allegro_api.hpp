// Copyright (c) 2022 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ALLEGRO_API__ALLEGRO_API_HPP_
#define ALLEGRO_API__ALLEGRO_API_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>


namespace allegro
{

class AllegroHand
{
public:
  AllegroHand();
  virtual ~AllegroHand();
  void init(const std::string& port, int id = 0);
  bool get_state(std::vector<double>& measured_position,
                 std::vector<double>& measured_velocity,
                 std::chrono::duration<long double> timeout = std::chrono::seconds(-1));
  void set_torque(const std::vector<double>& torque);
  double get_torque_limit(const std::string& joint_name);
  void set_torque_limit(const std::string& joint_name, double limit);
  std::vector<std::string> get_joint_names();
  void get_imu(std::vector<double>& quaternion);
  std::string get_serial();
  std::string get_hand_version();
  std::string get_firmware_version();
  std::string get_hand_type();
  double get_finger_temperature(int id);
  double get_hand_temperature();
  void set_device_id(uint8_t id);

private:
  class Implementation;
  std::unique_ptr<Implementation> impl_;
};

}  // namespace allegro

#endif  // ALLEGRO_API__ALLEGRO_API_HPP_
