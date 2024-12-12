// Copyright (c) 2024 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ALLEGRO_API_IMPLEMENTATION_HPP_
#define ALLEGRO_API_IMPLEMENTATION_HPP_

#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "allegro_api/allegro_api.hpp"

#define NUM_OF_FINGERS 4
#define NUM_OF_JOINTS 4 * NUM_OF_FINGERS

namespace allegro
{

class AllegroHand::Implementation
{
public:
  Implementation();
  virtual ~Implementation();
  void init(const std::string& port, int id = 0);
  bool get_state(std::vector<double>& measured_position,
                 std::vector<double>& measured_velocity,
                 std::chrono::duration<long double> timeout = std::chrono::seconds(-1));
  void get_velocity(std::vector<double>& measured_velocity);
  void set_torque(const std::vector<double>& torque);
  void get_imu(std::vector<double>& quaternion);
  double get_torque_limit(const std::string& joint_name);
  void set_torque_limit(const std::string& joint_name, double limit);
  std::vector<std::string> get_joint_names();
  void run();
  void update();
  void init_can();
  void can_servo(bool enable);
  void can_set_torque(int index, const std::vector<int16_t>& torque);
  void can_set_position(int index, const std::vector<int16_t>& position);
  void can_set_period(const std::vector<int16_t>& period);
  void can_set_device_id(uint8_t id);
  void can_set_rs485_baudrate(uint32_t rate);
  void can_request_information();
  void can_request_serial();
  void can_request_position(int index);
  void can_request_imu();
  void can_request_temperature(int index);

  std::vector<double> measured_position_;
  std::vector<double> measured_position_internal_;
  std::vector<double> measured_velocity_;
  std::vector<double> torque_;
  std::vector<double> temperature_;
  std::vector<double> imu_;
  double hand_temperature_;
  std::string serial_;
  std::string port_;
  std::string hand_version_;
  std::string hand_type_;
  std::string firmware_version_;
  uint8_t status_;
  bool status_servo_;
  double torque_constant_;
  double input_voltage_;
  double pwm_max_global_;
  std::vector<double> pwm_max_;
  int position_get_flag_;
  std::vector<int> encoder_offset_;
  std::vector<int> encoder_direction_;
  std::vector<int> motor_direction_;
  std::vector<std::vector<int16_t>> pwm_;
  std::vector<int16_t> period_;
  std::map<std::string, int> joints_;

  int socket_;
  int device_id_;
  std::unique_ptr<std::thread> main_thread_;
  bool is_running_;
  std::mutex get_mutex_;
  std::mutex set_mutex_;
  std::condition_variable get_condition_;
  bool state_available_;
  double t0_;
  double dt_;

  double pwm_limit_roll_ = 250.0*1.5;
  double pwm_limit_near_ = 450.0*1.5;
  double pwm_limit_middle_ = 300.0*1.5;
  double pwm_limit_far_ = 190.0*1.5;
  double pwm_limit_thumb_roll_ = 350.0*1.5;
  double pwm_limit_thumb_near_ = 270.0*1.5;
  double pwm_limit_thumb_middle_ = 180.0*1.5;
  double pwm_limit_thumb_far_ = 180.0*1.5;
  double pwm_limit_global_8v_ = 800.0;  // maximum: 1200
  double pwm_limit_global_24v_ = 500.0;
  double pwm_limit_global_12v_ = 1200.0;
};

}  // namespace allegro

#endif  // ALLEGRO_API_IMPLEMENTATION_HPP_
