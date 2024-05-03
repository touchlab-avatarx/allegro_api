// Copyright (c) 2022 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ALLEGRO_API__ALLEGRO_API_HPP_
#define ALLEGRO_API__ALLEGRO_API_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
  void get_velocity(std::vector<double>& measured_velocity);
  void set_torque(const std::vector<double>& torque);
  void get_imu(std::vector<double>& quaternion);
  std::string get_serial();
  std::string get_hand_version();
  std::string get_firmware_version();
  std::string get_hand_type();
  double get_finger_temperature(int id);
  double get_hand_temperature();

protected:
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

  int socket_;
  int can_id_;
  std::unique_ptr<std::thread> main_thread_;
  bool is_running_;
  std::mutex get_mutex_;
  std::mutex set_mutex_;
  std::condition_variable get_condition_;
  bool state_available_;
};

}  // namespace allegro

#endif  // ALLEGRO_API__ALLEGRO_API_HPP_
