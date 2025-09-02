// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ALLEGRO_API__ALLEGRO_API_HPP_
#define ALLEGRO_API__ALLEGRO_API_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace allegro
{

/**
 * @class AllegroHand
 * @brief Class for interfacing with the Allegro Hand.
 *
 * This class provides methods to initialize, control, and retrieve data from the Allegro Hand.
 */
class AllegroHand
{
public:
  /**
   * @brief Construct a new AllegroHand object.
   */
  AllegroHand();

  /**
   * @brief Destroy the AllegroHand object.
   */
  virtual ~AllegroHand();

  /**
   * @brief Initialize the Allegro Hand.
   *
   * @param port The CAN interface name to connect to the hand (e.g., "can0").
   * @param id The device CAN ID (default is 0).
   */
  void init(const std::string& port, int id = 0);

  /**
   * @brief Get the current state of the hand.
   *
   * @param measured_position Vector to store the measured joint positions (rad).
   * @param measured_velocity Vector to store the measured joint velocities (rad/s).
   * @param timeout Timeout duration for the operation (default is infinite).
   * @return true if the state was successfully retrieved, false otherwise.
   */
  bool get_state(std::vector<double>& measured_position,
                 std::vector<double>& measured_velocity,
                 std::chrono::duration<long double> timeout = std::chrono::seconds(-1));

  /**
   * @brief Set the torque for the joints.
   *
   * @param torque Vector of torque values to apply to the joints (N).
   */
  void set_torque(const std::vector<double>& torque);

  /**
   * @brief Get the torque limit for a specific joint.
   *
   * @param joint_name The name of the joint.
   * @return The torque limit for the specified joint (N).
   */
  double get_torque_limit(const std::string& joint_name);

  /**
   * @brief Set the torque limit for a specific joint.
   *
   * @param joint_name The name of the joint.
   * @param limit The new torque limit for the joint (N).
   */
  void set_torque_limit(const std::string& joint_name, double limit);

  /**
   * @brief Get the names of all joints in the hand.
   *
   * @return A vector of joint names.
   */
  std::vector<std::string> get_joint_names();

  /**
   * @brief Get the IMU data from the hand.
   *
   * @param quaternion Vector to store the IMU quaternion data.
   */
  void get_imu(std::vector<double>& quaternion);

  /**
   * @brief Get the serial number of the hand.
   *
   * @return The serial number as a string.
   */
  std::string get_serial();

  /**
   * @brief Get the version of the hand hardware.
   *
   * @return The hand version as a string.
   */
  std::string get_hand_version();

  /**
   * @brief Get the firmware version of the hand.
   *
   * @return The firmware version as a string.
   */
  std::string get_firmware_version();

  /**
   * @brief Get the type of the hand.
   *
   * @return The hand type as a string.
   */
  std::string get_hand_type();

  /**
   * @brief Get the temperature of a specific finger.
   *
   * @param id The ID of the finger.
   * @return The temperature of the specified finger (°C).
   */
  double get_finger_temperature(int id);

  /**
   * @brief Get the overall temperature of the hand.
   *
   * @return The temperature of the hand (°C).
   */
  double get_hand_temperature();

  /**
   * @brief Set the device CAN ID of the hand.
   *
   * @param id The new device CAN ID.
   */
  void set_device_id(uint8_t id);

private:
  /**
   * @class Implementation
   * @brief Private implementation class for the AllegroHand.
   */
  class Implementation;

  /// Unique pointer to the private implementation.
  std::unique_ptr<Implementation> impl_;
};

}  // namespace allegro

#endif  // ALLEGRO_API__ALLEGRO_API_HPP_
