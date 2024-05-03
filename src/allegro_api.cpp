// Copyright (c) 2022 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <exception>

#include <allegro_api/allegro_api.hpp>
#include <string.h>


using namespace std::chrono_literals;

#define THROW(m)                                                                                   \
  {                                                                                                \
    std::stringstream ss;                                                                          \
    ss << m;                                                                                       \
    throw std::runtime_error(ss.str());                                                            \
  }

#define INFO(x) std::cout << x << "\n";
#define WARNING(x) std::cout << "\033[1;32m[EXOTica]:\033[0m \033[33m" << x << "\033[0m\n";
#define WARNING_THROTTLE(t, x)                                                                     \
  {                                                                                                \
    static auto t0 = std::chrono::high_resolution_clock::now();                                    \
    if (std::chrono::high_resolution_clock::now() - t0 > t) {                                      \
                                                                                                   \
      t0 = std::chrono::high_resolution_clock::now();                                              \
      WARNING(x);                                                                                  \
    }                                                                                              \
  }

#define NUM_OF_FINGERS 4
#define NUM_OF_JOINTS 4 * NUM_OF_FINGERS

//  Define CAN Command
#define ID_CMD_SYSTEM_ON                0x40
#define ID_CMD_SYSTEM_OFF               0x41
#define ID_CMD_SET_TORQUE               0x60
#define ID_CMD_SET_TORQUE_1             (ID_CMD_SET_TORQUE+0)
#define ID_CMD_SET_TORQUE_2             (ID_CMD_SET_TORQUE+1)
#define ID_CMD_SET_TORQUE_3             (ID_CMD_SET_TORQUE+2)
#define ID_CMD_SET_TORQUE_4             (ID_CMD_SET_TORQUE+3)
#define ID_CMD_SET_POSE_1               0xE0
#define ID_CMD_SET_POSE_2               0xE1
#define ID_CMD_SET_POSE_3               0xE2
#define ID_CMD_SET_POSE_4               0xE3
#define ID_CMD_SET_PERIOD               0x81
#define ID_CMD_CONFIG                   0x68

////////////////////////////////////////////////
//  Define CAN Data Reqeust (RTR)
#define ID_RTR_HAND_INFO                0x80
#define ID_RTR_SERIAL                   0x88
#define ID_RTR_FINGER_POSE              0x20
#define ID_RTR_FINGER_POSE_1            (ID_RTR_FINGER_POSE+0)
#define ID_RTR_FINGER_POSE_2            (ID_RTR_FINGER_POSE+1)
#define ID_RTR_FINGER_POSE_3            (ID_RTR_FINGER_POSE+2)
#define ID_RTR_FINGER_POSE_4            (ID_RTR_FINGER_POSE+3)
#define ID_RTR_IMU_DATA                 0x30
#define ID_RTR_TEMPERATURE              0x38
#define ID_RTR_TEMPERATURE_1            (ID_RTR_TEMPERATURE+0)
#define ID_RTR_TEMPERATURE_2            (ID_RTR_TEMPERATURE+1)
#define ID_RTR_TEMPERATURE_3            (ID_RTR_TEMPERATURE+2)
#define ID_RTR_TEMPERATURE_4            (ID_RTR_TEMPERATURE+3)

// PWM limits
#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5
#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5
#define PWM_LIMIT_GLOBAL_8V 800.0  // maximum: 1200
#define PWM_LIMIT_GLOBAL_24V 500.0
#define PWM_LIMIT_GLOBAL_12V 1200.0

namespace allegro
{

// Joint names
enum JointName
{
    JOINTNAME_INDEX_0,
    JOINTNAME_INDEX_1,
    JOINTNAME_INDEX_2,
    JOINTNAME_INDEX_3,
    JOINTNAME_MIDDLE_0,
    JOINTNAME_MIDDLE_1,
    JOINTNAME_MIDDLE_2,
    JOINTNAME_MIDDLE_3,
    JOINTNAME_PINKY_0,
    JOINTNAME_PINKY_1,
    JOINTNAME_PINKY_2,
    JOINTNAME_PINKY_3,
    JOINTNAME_THUMB_0,
    JOINTNAME_THUMB_1,
    JOINTNAME_THUMB_2,
    JOINTNAME_THUMB_3,
    DOF_JOINTS
};

inline canid_t standard_frame_id(canid_t frame_id, canid_t device_id)
{
  return (frame_id << 2) | device_id;
}

inline canid_t rtr_frame_id(canid_t frame_id, canid_t device_id)
{
  return ((frame_id) << 2) | device_id | CAN_RTR_FLAG;
}

bool can_read(int socket, can_frame& frame)
{
  return (::read(socket, &frame, sizeof(frame)) == sizeof(frame));
}

void can_write(int socket, can_frame& frame)
{
  if (::write(socket, &frame, sizeof(frame)) != sizeof(frame))
  {
    WARNING_THROTTLE(1.0s, "Can't send data frame");
  }
}

AllegroHand::AllegroHand() : is_running_(false), socket_(0)
{
  measured_position_.assign(NUM_OF_JOINTS, 0.0);
  measured_position_internal_.assign(NUM_OF_JOINTS, 0.0);
  measured_velocity_.assign(NUM_OF_JOINTS, 0.0);
  torque_.assign(NUM_OF_JOINTS, 0.0);
  temperature_.assign(NUM_OF_FINGERS, 0.0);
  pwm_max_.assign(NUM_OF_JOINTS, 0.0);
  imu_.assign(3, 0.0);
  hand_temperature_ = 0.0;
  hand_version_ = "00";
  encoder_offset_.assign(NUM_OF_JOINTS, 0.0);
  encoder_direction_.assign(NUM_OF_JOINTS, 1.0);
  motor_direction_.assign(NUM_OF_JOINTS, 1.0);
  for (int i = 0; i < NUM_OF_FINGERS; ++i)
    pwm_.push_back({0, 0, 0, 0});
  period_ = { 3, 0, 0 };  // state, imu, temperature (in ms)
}

AllegroHand::~AllegroHand()
{
  is_running_ = false;
  if (socket_)
  {
    can_frame frame;
    can_servo(false);
    ::close(socket_);
    std::this_thread::sleep_for(0.5s);
  }
  if (main_thread_)
    main_thread_->join();
}

void AllegroHand::init(const std::string& port, int id)
{
  port_ = port;
  can_id_ = id;
  init_can();

  can_servo(false);
  std::this_thread::sleep_for(100us);

  can_request_information();
  std::this_thread::sleep_for(100us);

  can_request_serial();
  std::this_thread::sleep_for(100us);

  can_set_period(period_);
  can_servo(true);

  is_running_ = true;
  main_thread_.reset(new std::thread([&]() {run();}));

  std::this_thread::sleep_for(0.5s);
}

bool AllegroHand::get_state(std::vector<double>& measured_position,
                 std::vector<double>& measured_velocity,
                 std::chrono::duration<long double> timeout)
{
  std::unique_lock<std::mutex> guard(get_mutex_);
  if (timeout > 0s) {
    if (get_condition_.wait_until(
          guard, std::chrono::system_clock::now() + timeout,
          [&] { return state_available_ == true; })) {
      measured_position = measured_position_;
      measured_velocity = measured_velocity_;
      state_available_ = false;
      guard.unlock();
      return true;
    } else {
      guard.unlock();
      return false;
    }
  } else {
    measured_position = measured_position_;
    measured_velocity = measured_velocity_;
    guard.unlock();
    return true;
  }
}

void AllegroHand::set_torque(const std::vector<double>& torque)
{
  if (torque.size() == torque_.size())
  {
    std::unique_lock<std::mutex> guard(set_mutex_);
    torque_ = torque;
    guard.unlock();
  } else {
    THROW("Invalid torque command vector size. Expecting " << torque_.size() << " got " <<
          torque.size());
  }
}

void AllegroHand::get_imu(std::vector<double>& quaternion)
{
  std::unique_lock<std::mutex> guard(get_mutex_);
  quaternion = imu_;
  guard.unlock();
}

std::string AllegroHand::get_serial()
{
  return serial_;
}

std::string AllegroHand::get_hand_version()
{
  return hand_version_;
}

std::string AllegroHand::get_firmware_version()
{
  return firmware_version_;
}

std::string AllegroHand::get_hand_type()
{
  return hand_type_;
}

double AllegroHand::get_finger_temperature(int id)
{
  if (id < 0 || id >= NUM_OF_FINGERS)
    return -1.0;
  return temperature_[id];
}

double AllegroHand::get_hand_temperature()
{
  return hand_temperature_;
}

void AllegroHand::run()
{
  while (is_running_)
  {
    update();
    std::this_thread::yield();
  }
}

double clamp(double value, double lower, double upper)
{
  return std::min(upper, std::max(lower, value));
}

void AllegroHand::update()
{
  can_frame frame;
  position_get_flag_ = 0;
  while (is_running_)
  {
    if (can_read(socket_, frame))
    {
      // Parse frame
      canid_t can_id = (frame.can_id & 0xfffffffc) >> 2;
      switch (can_id)
      {
        case ID_RTR_HAND_INFO:
        {
          std::unique_lock<std::mutex> guard(get_mutex_);
          {
            std::stringstream ss;
            ss << std::to_string(frame.data[1]) << "." << std::to_string(frame.data[0]);
            hand_version_ = ss.str();
          }
          {
            std::stringstream ss;
            ss << std::to_string(frame.data[3]) << "." << std::to_string(frame.data[2]);
            firmware_version_ = ss.str();
          }
          hand_type_ = (frame.data[4] == 0 ? "right" : "left");
          hand_temperature_ = frame.data[5];
          status_ = frame.data[6];
          status_servo_ = (status_ & 0x01 ? true : false);
          if ((status_ & 0x02) != 0) THROW("High temperature fault");
          if ((status_ & 0x04) != 0) THROW("Internal communication fault");
          uint8_t hand_version = frame.data[1];
          if (hand_version == 4)
          {
              // v4
              torque_constant_ = 1200.0;
              input_voltage_ = 12.0;
              pwm_max_global_ = PWM_LIMIT_GLOBAL_12V;
          } else {
              // v3
              torque_constant_ = 800.0;
              input_voltage_ = 8.0;
              pwm_max_global_ = PWM_LIMIT_GLOBAL_8V;
          }

          pwm_max_[JOINTNAME_INDEX_0] = std::min(pwm_max_global_, PWM_LIMIT_ROLL);
          pwm_max_[JOINTNAME_INDEX_1] = std::min(pwm_max_global_, PWM_LIMIT_NEAR);
          pwm_max_[JOINTNAME_INDEX_2] = std::min(pwm_max_global_, PWM_LIMIT_MIDDLE);
          pwm_max_[JOINTNAME_INDEX_3] = std::min(pwm_max_global_, PWM_LIMIT_FAR);

          pwm_max_[JOINTNAME_MIDDLE_0] = std::min(pwm_max_global_, PWM_LIMIT_ROLL);
          pwm_max_[JOINTNAME_MIDDLE_1] = std::min(pwm_max_global_, PWM_LIMIT_NEAR);
          pwm_max_[JOINTNAME_MIDDLE_2] = std::min(pwm_max_global_, PWM_LIMIT_MIDDLE);
          pwm_max_[JOINTNAME_MIDDLE_3] = std::min(pwm_max_global_, PWM_LIMIT_FAR);

          pwm_max_[JOINTNAME_PINKY_0] = std::min(pwm_max_global_, PWM_LIMIT_ROLL);
          pwm_max_[JOINTNAME_PINKY_1] = std::min(pwm_max_global_, PWM_LIMIT_NEAR);
          pwm_max_[JOINTNAME_PINKY_2] = std::min(pwm_max_global_, PWM_LIMIT_MIDDLE);
          pwm_max_[JOINTNAME_PINKY_3] = std::min(pwm_max_global_, PWM_LIMIT_FAR);

          pwm_max_[JOINTNAME_THUMB_0] = std::min(pwm_max_global_, PWM_LIMIT_THUMB_ROLL);
          pwm_max_[JOINTNAME_THUMB_1] = std::min(pwm_max_global_, PWM_LIMIT_THUMB_NEAR);
          pwm_max_[JOINTNAME_THUMB_2] = std::min(pwm_max_global_, PWM_LIMIT_THUMB_MIDDLE);
          pwm_max_[JOINTNAME_THUMB_3] = std::min(pwm_max_global_, PWM_LIMIT_THUMB_FAR);
          guard.unlock();
          break;
        }
        case ID_RTR_SERIAL:
        {
          std::stringstream ss;
          ss << "SAH0" << hand_version_[0] << "0"
            << static_cast<char>(frame.data[0])
            << static_cast<char>(frame.data[1])
            << static_cast<char>(frame.data[2])
            << static_cast<char>(frame.data[3])
            << static_cast<char>(frame.data[4])
            << static_cast<char>(frame.data[5])
            << static_cast<char>(frame.data[6])
            << static_cast<char>(frame.data[7]) << std::ends;
          std::unique_lock<std::mutex> guard(get_mutex_);
          serial_ = ss.str();
          guard.unlock();
          break;
        }
        case ID_RTR_FINGER_POSE_1:
        case ID_RTR_FINGER_POSE_2:
        case ID_RTR_FINGER_POSE_3:
        case ID_RTR_FINGER_POSE_4:
        {
            int index = (can_id & 0x00000007);
            int16_t tmppos[NUM_OF_FINGERS];
            tmppos[0] = static_cast<int16_t>(frame.data[0] | (frame.data[1] << 8));
            tmppos[1] = static_cast<int16_t>(frame.data[2] | (frame.data[3] << 8));
            tmppos[2] = static_cast<int16_t>(frame.data[4] | (frame.data[5] << 8));
            tmppos[3] = static_cast<int16_t>(frame.data[6] | (frame.data[7] << 8));

            int offset = index * 4;

            measured_position_internal_[offset + 0] = static_cast<double>(tmppos[0]) *
                                                      (333.3 / 65536.0) * ( M_PI / 180.0);
            measured_position_internal_[offset + 1] = static_cast<double>(tmppos[1]) *
                                                      (333.3 / 65536.0) * ( M_PI / 180.0);
            measured_position_internal_[offset + 2] = static_cast<double>(tmppos[2]) *
                                                      (333.3 / 65536.0) * ( M_PI / 180.0);
            measured_position_internal_[offset + 3] = static_cast<double>(tmppos[3]) *
                                                      (333.3 / 65536.0) * ( M_PI / 180.0);

            position_get_flag_ |= (0x01 << (index));
        }
            break;
        case ID_RTR_IMU_DATA:
        {
          int16_t tmppos[4];
          tmppos[0] = static_cast<int16_t>(frame.data[0] | (frame.data[1] << 8));
          tmppos[1] = static_cast<int16_t>(frame.data[2] | (frame.data[3] << 8));
          tmppos[2] = static_cast<int16_t>(frame.data[4] | (frame.data[5] << 8));
          tmppos[3] = static_cast<int16_t>(frame.data[6] | (frame.data[7] << 8));
          std::unique_lock<std::mutex> guard(get_mutex_);
          imu_[0] = static_cast<double>(tmppos[0]);
          imu_[1] = static_cast<double>(tmppos[1]);
          imu_[2] = static_cast<double>(tmppos[2]);
          imu_[3] = static_cast<double>(tmppos[3]);
          guard.unlock();
        }
            break;
        case ID_RTR_TEMPERATURE_1:
        case ID_RTR_TEMPERATURE_2:
        case ID_RTR_TEMPERATURE_3:
        case ID_RTR_TEMPERATURE_4:
        {
            int index = (can_id & 0x00000007);
            std::unique_lock<std::mutex> guard(get_mutex_);
            temperature_[index] = static_cast<int>(frame.data[0]) |
                          static_cast<int>(frame.data[1] << 8 ) |
                          static_cast<int>(frame.data[2] << 16) |
                          static_cast<int>(frame.data[3] << 24);
            guard.unlock();
        }
            break;
        default:
            WARNING_THROTTLE(1.0s, "Unknown CAN frame " << can_id << "(len: " <<
                             frame.can_dlc << ")");
            break;
    }
      // End parsing frame
    } else {
      break;
    }
  }

  bool is_state_ready = position_get_flag_ == (0x01 | 0x02 | 0x04 | 0x08);
  if (!is_state_ready) return;

  // Update state
  {
    std::unique_lock<std::mutex> guard(get_mutex_);
    double dt = period_[0] * 1e-3;
    for (int i = 0; i < NUM_OF_JOINTS; ++i)
    {
      measured_velocity_[i] = (measured_position_internal_[i] - measured_position_[i]) / dt;
      measured_position_[i] = measured_position_internal_[i];
    }
    state_available_ = true;
    get_condition_.notify_all();
    guard.unlock();
  }

  // Update torque commands
  {
    std::unique_lock<std::mutex> guard(set_mutex_);
    for (int i = 0; i < NUM_OF_FINGERS; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        int ii = i * 4 + j;
        pwm_[i][j] = static_cast<int16_t>(
          clamp(torque_[ii] * torque_constant_, -pwm_max_[ii], pwm_max_[ii]));
      }
    }
    guard.unlock();

    for (int i = 0; i < NUM_OF_FINGERS; ++i)
      can_set_torque(i, pwm_[i]);
  }
}

void AllegroHand::init_can()
{
  struct sockaddr_can address;
  struct ifreq interface_request;

  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    THROW("Can't create CAN socket");
  }

  snprintf(interface_request.ifr_name, IFNAMSIZ, "%s", port_.c_str());
  if (ioctl(socket_, SIOCGIFINDEX, &interface_request) != 0)
  {
    THROW("Can't find interface " << port_);
  }

  memset(&address, 0, sizeof(address));
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;

  int ret = bind(socket_, (struct sockaddr*)&address, sizeof(address));
  if (ret < 0)
  {
    THROW("Can't bind CAN socket " << port_ << " " << ret);
  }

  fcntl(socket_, F_SETFL, O_NONBLOCK);
}

void AllegroHand::can_servo(bool enable)
{
  can_frame frame;
  frame.can_id = standard_frame_id(enable ? ID_CMD_SYSTEM_ON : ID_CMD_SYSTEM_OFF, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

void AllegroHand::can_set_torque(int index, const std::vector<int16_t>& torque)
{
  if (index < 0 || index >= NUM_OF_FINGERS) THROW("Invalid finger index for setting torque");
  can_frame frame;
  frame.data[0] = (unsigned char)( (torque[0]     ) & 0x00ff);
  frame.data[1] = (unsigned char)( (torque[0] >> 8) & 0x00ff);
  frame.data[2] = (unsigned char)( (torque[1]     ) & 0x00ff);
  frame.data[3] = (unsigned char)( (torque[1] >> 8) & 0x00ff);
  frame.data[4] = (unsigned char)( (torque[2]     ) & 0x00ff);
  frame.data[5] = (unsigned char)( (torque[2] >> 8) & 0x00ff);
  frame.data[6] = (unsigned char)( (torque[3]     ) & 0x00ff);
  frame.data[7] = (unsigned char)( (torque[3] >> 8) & 0x00ff);
  frame.can_id = standard_frame_id(ID_CMD_SET_TORQUE_1 + index, can_id_);
  frame.can_dlc = 8;
  can_write(socket_, frame);
}

void AllegroHand::can_set_position(int index, const std::vector<int16_t>& position)
{
  if (index < 0 || index >= NUM_OF_FINGERS) THROW("Invalid finger index for setting position");
  can_frame frame;
  frame.data[0] = (unsigned char)( (position[0]     ) & 0x00ff);
  frame.data[1] = (unsigned char)( (position[0] >> 8) & 0x00ff);
  frame.data[2] = (unsigned char)( (position[1]     ) & 0x00ff);
  frame.data[3] = (unsigned char)( (position[1] >> 8) & 0x00ff);
  frame.data[4] = (unsigned char)( (position[2]     ) & 0x00ff);
  frame.data[5] = (unsigned char)( (position[2] >> 8) & 0x00ff);
  frame.data[6] = (unsigned char)( (position[3]     ) & 0x00ff);
  frame.data[7] = (unsigned char)( (position[3] >> 8) & 0x00ff);
  frame.can_id = standard_frame_id(ID_CMD_SET_POSE_1 + index, can_id_);
  frame.can_dlc = 8;
  can_write(socket_, frame);
}

void AllegroHand::can_set_period(const std::vector<int16_t>& period)
{
  if (period.size() != 3) THROW("Invalid period vector size");
  can_frame frame;
  frame.can_id = standard_frame_id(ID_CMD_SET_PERIOD, can_id_);
  frame.can_dlc = 6;
  frame.data[0] = (uint8_t)( (period[0]     ) & 0x00ff);
  frame.data[1] = (uint8_t)( (period[0] >> 8) & 0x00ff);
  frame.data[2] = (uint8_t)( (period[1]     ) & 0x00ff);
  frame.data[3] = (uint8_t)( (period[1] >> 8) & 0x00ff);
  frame.data[4] = (uint8_t)( (period[2]     ) & 0x00ff);
  frame.data[5] = (uint8_t)( (period[2] >> 8) & 0x00ff);
  can_write(socket_, frame);
}

void AllegroHand::can_set_device_id(uint8_t id)
{
  can_frame frame;
  frame.can_id = standard_frame_id(ID_CMD_CONFIG, can_id_);
  frame.can_dlc = 6;
  frame.data[0] = id | 0x80;
  frame.data[1] = 0x00;
  frame.data[5] = 0x00;
  can_write(socket_, frame);
}

void AllegroHand::can_set_rs485_baudrate(uint32_t rate)
{
  can_frame frame;
  frame.can_id = standard_frame_id(ID_CMD_CONFIG, can_id_);
  frame.can_dlc = 6;
  frame.data[0] = 0x00;
  frame.data[1] = (uint8_t)( (rate      ) & 0x000000ff);
  frame.data[2] = (uint8_t)( (rate >> 8 ) & 0x000000ff);
  frame.data[3] = (uint8_t)( (rate >> 16) & 0x000000ff);
  frame.data[4] = (uint8_t)( (rate >> 24) & 0x000000ff) | 0x80;
  frame.data[5] = 0x00;
  can_write(socket_, frame);
}

void AllegroHand::can_request_information()
{
  can_frame frame;
  frame.can_id = rtr_frame_id(ID_RTR_HAND_INFO, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

void AllegroHand::can_request_serial()
{
  can_frame frame;
  frame.can_id = rtr_frame_id(ID_RTR_SERIAL, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

void AllegroHand::can_request_position(int index)
{
  if (index < 0 || index >= NUM_OF_FINGERS) THROW("Invalid finger index for requesting position");
  can_frame frame;
  frame.can_id = rtr_frame_id(ID_RTR_FINGER_POSE + index, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

void AllegroHand::can_request_imu()
{
  can_frame frame;
  frame.can_id = rtr_frame_id(ID_RTR_IMU_DATA, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

void AllegroHand::can_request_temperature(int index)
{
  if (index < 0 || index >= NUM_OF_FINGERS) THROW("Invalid finger index for requesting position");
  can_frame frame;
  frame.can_id = rtr_frame_id(ID_RTR_TEMPERATURE + index, can_id_);
  frame.can_dlc = 0;
  can_write(socket_, frame);
}

}  // namespace allegro
