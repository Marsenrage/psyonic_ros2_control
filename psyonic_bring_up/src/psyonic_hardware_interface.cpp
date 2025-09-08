#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "msg_dexmobile/msg/stamped_float64_array.hpp"
#include "psyonic_bring_up/psyonic_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace psyonic_hardware_interface
{

hardware_interface::CallbackReturn PsyonicHandHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  print_hardware_info(info);

  // Parse parameters from URDF
  try {
    device_name_ = info_.hardware_parameters.at("device_name");
    baud_rate_ = std::stoul(info_.hardware_parameters.at("baud_rate"));
    hand_address_ = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.at("hand_address")));
    timeout_ = std::stod(info_.hardware_parameters.at("timeout"));
    
    // Parse command mode
    std::string cmd_mode_str = info_.hardware_parameters.at("command_mode");
    if (cmd_mode_str == "POSITION") {
      command_mode_ = Command::POSITION;
    } else if (cmd_mode_str == "VELOCITY") {
      command_mode_ = Command::VELOCITY;
    } else if (cmd_mode_str == "CURRENT") {
      command_mode_ = Command::CURRENT;
    } else if (cmd_mode_str == "DUTY") {
      command_mode_ = Command::DUTY;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
        "Invalid command mode: %s. Using POSITION as default.", cmd_mode_str.c_str());
      command_mode_ = Command::POSITION;
    }
    command_mode_ = Command::POSITION;
    // Parse reply mode (0, 1, or 2)
    reply_mode_ = static_cast<uint8_t>(std::stoul(info_.hardware_parameters.at("reply_model")));
    if (reply_mode_ > 2) {
      RCLCPP_WARN(rclcpp::get_logger("PsyonicHandHardware"), 
        "Invalid reply mode: %d. Using 0 as default.", reply_mode_);
      reply_mode_ = 0;
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Error parsing parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint configuration
  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Expected 6 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage vectors
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_fsr_sensors_.resize(30, 0.0);  // 6 sensors per finger × 5 fingers

  // Validate each joint has required interfaces
  for (const auto & joint : info_.joints) {
    // Check for required command interfaces
    bool has_command_interface = false;
    for (const auto & interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION ||
          interface.name == hardware_interface::HW_IF_VELOCITY ||
          interface.name == hardware_interface::HW_IF_EFFORT) {
        has_command_interface = true;
        break;
      }
    }
    
    if (!has_command_interface) {
      RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
        "Joint '%s' has no valid command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check for required state interfaces
    bool has_position_state = false;
    bool has_velocity_state = false;
    bool has_effort_state = false;
    
    for (const auto & interface : joint.state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
        has_effort_state = true;
      }
    }
    
    // Position is always required
    if (!has_position_state) {
      RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
        "Joint '%s' has no position state interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Check if velocity state is needed based on reply mode
    if ((reply_mode_ == 1 || reply_mode_ == 2) && !has_velocity_state) {
      RCLCPP_WARN(rclcpp::get_logger("PsyonicHandHardware"), 
        "Joint '%s' has no velocity state interface but reply_mode %d provides velocity data", 
        joint.name.c_str(), reply_mode_);
    }
    
    // Check if effort state is needed based on reply mode
    if ((reply_mode_ == 0 || reply_mode_ == 2) && !has_effort_state) {
      RCLCPP_WARN(rclcpp::get_logger("PsyonicHandHardware"), 
        "Joint '%s' has no effort state interface but reply_mode %d provides current data", 
        joint.name.c_str(), reply_mode_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Successfully initialized PSYONIC hand hardware interface");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PsyonicHandHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "Configuring PSYONIC hand...");

  try {
    // Create the hand wrapper
    hand_wrapper_ = std::make_unique<AHWrapper>(hand_address_, baud_rate_, device_name_);
    
    // Create ROS2 node for publishing FSR data
    node_ = rclcpp::Node::make_shared("psyonic_fsr_publisher");
    fsr_publisher_ = node_->create_publisher<msg_dexmobile::msg::StampedFloat64Array>(
      "/psyonic_hand/fsr_sensors", 10);
    
    // Initialize FSR message
    fsr_msg_.header.frame_id = "psyonic_hand";
    fsr_msg_.data.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    fsr_msg_.data.layout.dim[0].label = "fsr_sensors";
    fsr_msg_.data.layout.dim[0].size = 30;
    fsr_msg_.data.layout.dim[0].stride = 30;
    fsr_msg_.data.layout.data_offset = 0;
    fsr_msg_.data.data.resize(30);
    
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
      "Created hand wrapper with address: 0x%02X, baud rate: %u", 
      hand_address_, baud_rate_);

    // Initialize FSR processing
    fsr_baselines_.resize(30, 0.0);
    fsr_calibrated_ = false;
    calibration_samples_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
      "FSR baseline calibration will start when hand is activated");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Failed to create hand wrapper: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Successfully configured PSYONIC hand hardware interface");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PsyonicHandHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "Activating PSYONIC hand...");

  if (!hand_wrapper_) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Hand wrapper not initialized");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Attempt to connect to the hand
  if (hand_wrapper_->connect() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Failed to connect to PSYONIC hand");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize with safe positions (all zeros - extended position)
  std::array<float, 6> safe_positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  // Send initial command to enter API mode
  int result = hand_wrapper_->read_write_once(safe_positions, command_mode_, reply_mode_);
  if (result != 0) {
    RCLCPP_WARN(rclcpp::get_logger("PsyonicHandHardware"), 
      "Initial communication failed, but continuing...");
  }

  // Update initial state from hand
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = static_cast<double>(hand_wrapper_->hand.pos[i]) * M_PI / 180.0;
    hw_velocities_[i] = static_cast<double>(hand_wrapper_->hand.vel[i]) * M_PI / 180.0;
    hw_efforts_[i] = static_cast<double>(hand_wrapper_->hand.cur[i]);
  }

  // Update FSR sensors
  for (size_t i = 0; i < hw_fsr_sensors_.size() && i < 30; ++i) {
    hw_fsr_sensors_[i] = static_cast<double>(hand_wrapper_->hand.fsr[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Successfully activated PSYONIC hand hardware interface");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PsyonicHandHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "Deactivating PSYONIC hand...");

  if (hand_wrapper_) {
    // Send safe position before deactivating
    std::array<float, 6> safe_positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    hand_wrapper_->read_write_once(safe_positions, Command::POSITION, reply_mode_);
    
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
      "Sent safe position command before deactivation");
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Successfully deactivated PSYONIC hand hardware interface");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PsyonicHandHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export joint state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    
    for (const auto & interface : joint.state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_positions_[i]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_velocities_[i]);
      } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(joint.name, interface.name, &hw_efforts_[i]);
      }
    }
  }

  // Export FSR sensor interfaces
  for (size_t i = 0; i < info_.sensors.size(); ++i) {
    const auto & sensor = info_.sensors[i];
    
    for (size_t j = 0; j < sensor.state_interfaces.size(); ++j) {
      const auto & interface = sensor.state_interfaces[j];
      size_t fsr_index = i * 6 + j;  // Assuming 6 sensors per finger
      
      if (fsr_index < hw_fsr_sensors_.size()) {
        state_interfaces.emplace_back(sensor.name, interface.name, &hw_fsr_sensors_[fsr_index]);
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PsyonicHandHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export joint command interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    
    for (const auto & interface : joint.command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION ||
          interface.name == hardware_interface::HW_IF_VELOCITY ||
          interface.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(joint.name, interface.name, &hw_commands_[i]);
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::return_type PsyonicHandHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!hand_wrapper_) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Hand wrapper not initialized");
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO_ONCE(rclcpp::get_logger("PsyonicHandHardware"), 
  "Joint mapping - Raw degrees: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
  hand_wrapper_->hand.pos[0], hand_wrapper_->hand.pos[1], hand_wrapper_->hand.pos[2],
  hand_wrapper_->hand.pos[3], hand_wrapper_->hand.pos[4], hand_wrapper_->hand.pos[5]);

  // The read operation is actually performed in write() due to the API design
  // Here we just update our state vectors with the latest data from the hand
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = static_cast<double>(hand_wrapper_->hand.pos[i]) * M_PI / 180.0;
    hw_velocities_[i] = static_cast<double>(hand_wrapper_->hand.vel[i]) * M_PI / 180.0;
    hw_efforts_[i] = static_cast<double>(hand_wrapper_->hand.cur[i]);
  }

  RCLCPP_INFO_ONCE(rclcpp::get_logger("PsyonicHandHardware"), 
  "Joint mapping - Raw degrees: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
  hand_wrapper_->hand.pos[0], hand_wrapper_->hand.pos[1], hand_wrapper_->hand.pos[2],
  hand_wrapper_->hand.pos[3], hand_wrapper_->hand.pos[4], hand_wrapper_->hand.pos[5]);
 
  // Update FSR sensors
  for (size_t i = 0; i < hw_fsr_sensors_.size() && i < 30; ++i) {
    hw_fsr_sensors_[i] = static_cast<double>(hand_wrapper_->hand.fsr[i]);
    fsr_msg_.data.data[i] = hw_fsr_sensors_[i];
  }
  process_fsr_data();
  // Publish FSR data with timestamp
  if (fsr_publisher_) {
    fsr_msg_.header.stamp = node_->get_clock()->now();
    fsr_publisher_->publish(fsr_msg_);
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type PsyonicHandHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static int call_count = 0;
  // if (++call_count % 125 == 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
  //     "write() called %d times - Commands: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //     call_count, hw_commands_[0], hw_commands_[1], hw_commands_[2], 
  //     hw_commands_[3], hw_commands_[4], hw_commands_[5]);
  // }

  if (!hand_wrapper_) {
    RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
      "Hand wrapper not initialized");
    return hardware_interface::return_type::ERROR;
  }

  // Apply position feedback control - testing
  apply_position_feedback_control();
  apply_deadband_compensation();

  // Convert commands from radians to degrees and apply safety limits
  std::array<float, 6> cmd_array;
  for (size_t i = 0; i < 6; ++i) {
    double cmd_degrees = hw_commands_[i] * 180.0 / M_PI;
    
    if (std::isnan(cmd_degrees) || std::isinf(cmd_degrees)) {
      RCLCPP_ERROR(rclcpp::get_logger("PsyonicHandHardware"), 
        "Invalid command for joint %zu: %f", i, cmd_degrees);
      return hardware_interface::return_type::ERROR;
    }

    // Apply safety limits in degrees
    if (command_mode_ == Command::POSITION) {
      if (i == 4) { // thumb_q2 (flexor)
        cmd_degrees = std::max(0.0, std::min(150.0, cmd_degrees));
      } else if (i == 5) { // thumb_q1 (rotator) 
        cmd_degrees = std::max(-150.0, std::min(0.0, cmd_degrees));
      } else { // other fingers
        cmd_degrees = std::max(0.0, std::min(150.0, cmd_degrees));
      }
    }

    cmd_array[i] = static_cast<float>(cmd_degrees);
  }

  // DEBUG: Print converted commands
  // if (call_count % 125 == 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
  //     "Sending commands (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
  //     cmd_array[0], cmd_array[1], cmd_array[2], 
  //     cmd_array[3], cmd_array[4], cmd_array[5]);
  // }

  // Send command and read response
  int result = hand_wrapper_->read_write_once(cmd_array, command_mode_, reply_mode_);
  
  // DEBUG: Print communication result
  // if (call_count % 125 == 0) {
  //   RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
  //     "Communication result: %d", result);
  // }
  
  if (result != 0) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PsyonicHandHardware"), 
      *rclcpp::Clock::make_shared(), 1000,
      "Communication with hand failed with result: %d", result);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void PsyonicHandHardware::print_hardware_info(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "Hardware Info:");
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "  Name: %s", info.name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "  Type: %s", info.type.c_str());
  
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "  Parameters:");
  for (const auto & param : info.hardware_parameters) {
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
      "    %s: %s", param.first.c_str(), param.second.c_str());
  }
  
  RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "  Joints (%zu):", info.joints.size());
  for (const auto & joint : info.joints) {
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "    - %s", joint.name.c_str());
    
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "      Command interfaces:");
    for (const auto & cmd_interface : joint.command_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
        "        - %s", cmd_interface.name.c_str());
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "      State interfaces:");
    for (const auto & state_interface : joint.state_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
        "        - %s", state_interface.name.c_str());
    }
  }
  
  if (!info.sensors.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "  Sensors (%zu):", info.sensors.size());
    for (const auto & sensor : info.sensors) {
      RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), "    - %s", sensor.name.c_str());
    }
  }
}

}  // namespace psyonic_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  psyonic_hardware_interface::PsyonicHandHardware,
  hardware_interface::SystemInterface)