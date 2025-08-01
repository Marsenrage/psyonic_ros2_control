#pragma once
#include <array>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "psyonic_bring_up/hand.h"
#include "psyonic_bring_up/wrapper.h"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "msg_dexmobile/msg/stamped_float64_array.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace psyonic_hardware_interface
{

class PsyonicHandHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PsyonicHandHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware communication
  std::unique_ptr<AHWrapper> hand_wrapper_;

  // Publisher
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<msg_dexmobile::msg::StampedFloat64Array>::SharedPtr fsr_publisher_;
  msg_dexmobile::msg::StampedFloat64Array fsr_msg_;

  // Parameters
  std::string device_name_;
  uint32_t baud_rate_;
  double timeout_;
  uint8_t hand_address_;
  Command command_mode_;
  uint8_t reply_mode_;

  // Command and state storage (6 joints: index, middle, ring, pinky, thumb_flexor, thumb_rotator)
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_fsr_sensors_;  // 30 sensors (6 per finger × 5 fingers)

  // FSR processing
  std::vector<double> fsr_baselines_;  // Store baseline values for each sensor
  bool fsr_calibrated_;               // Track if baseline calibration is done
  size_t calibration_samples_;        // Number of samples for baseline calculation
  static constexpr size_t CALIBRATION_SAMPLE_COUNT = 250; // 2 seconds at 125Hz

  // Helper methods
  void print_hardware_info(const hardware_interface::HardwareInfo& info);

  // fsr conversion and calibration
  void process_fsr_data() {
    // During calibration phase, collect baseline data
    if (!fsr_calibrated_ && calibration_samples_ < CALIBRATION_SAMPLE_COUNT) {
      for (size_t i = 0; i < 30; ++i) {
        fsr_baselines_[i] += static_cast<double>(hand_wrapper_->hand.fsr[i]);
      }
      calibration_samples_++;
      
      // Finish calibration
      if (calibration_samples_ >= CALIBRATION_SAMPLE_COUNT) {
        for (size_t i = 0; i < 30; ++i) {
          fsr_baselines_[i] /= CALIBRATION_SAMPLE_COUNT; // Average
        }
        fsr_calibrated_ = true;
        RCLCPP_INFO(rclcpp::get_logger("PsyonicHandHardware"), 
          "FSR baseline calibration completed");
      }
      return;
    }
    
    // Process FSR data after calibration
    if (fsr_calibrated_) {
      for (size_t i = 0; i < 30; ++i) {
        double raw_value = static_cast<double>(hand_wrapper_->hand.fsr[i]);
        
        // Subtract baseline to get zero-referenced value
        double zero_ref_value = std::max(0.0, raw_value - fsr_baselines_[i]);
        
        // Convert to force using PSYONIC formulas
        double force_newtons = 0.0;
        if (zero_ref_value > 0) {
          // Convert ADC to voltage
          double voltage = zero_ref_value * 3.3 / 4096.0;
          
          // Convert to resistance (avoid division by zero)
          if (voltage > 0.001) { // Small threshold to avoid div by zero
            double resistance = 33000.0 / voltage + 10000.0;
            
            // Convert to force (using PSYONIC constants)
            force_newtons = 121591.0 / resistance + 0.878894;
          }
        }
        
        // Store processed force value
        hw_fsr_sensors_[i] = force_newtons;
        fsr_msg_.data.data[i] = force_newtons;
      }
    }
  }// process_fsr_data


  // PID controller for position correction
  std::vector<double> position_errors_;
  std::vector<double> integral_errors_;
  std::vector<double> previous_errors_;
  std::vector<double> corrected_commands_;

  // PID gains (tunable parameters)
  double kp_ = 0.5;  // Proportional gain
  double ki_ = 0.05;  // Integral gain  
  double kd_ = 0.01; // Derivative gain

  void apply_position_feedback_control() {   // pid control
    if (position_errors_.empty()) {
      position_errors_.resize(6, 0.0);
      integral_errors_.resize(6, 0.0);
      previous_errors_.resize(6, 0.0);
      corrected_commands_.resize(6, 0.0);
    }

    for (size_t i = 0; i < 6; ++i) {
      // Calculate position error (command - actual)
      double error = hw_commands_[i] - hw_positions_[i];
      
      // Update integral error with windup protection
      integral_errors_[i] += error;
      integral_errors_[i] = std::max(-0.1, std::min(0.1, integral_errors_[i])); // Limit integral windup
      
      // Calculate derivative error
      double derivative_error = error - previous_errors_[i];
      
      // Apply PID correction
      double correction = kp_ * error + ki_ * integral_errors_[i] + kd_ * derivative_error;
      
      // Apply correction to command
      corrected_commands_[i] = hw_commands_[i] + correction;
      
      // Store for next iteration
      previous_errors_[i] = error;
    }
  } //apply_position_feedback_control


  static constexpr double POSITION_DEADBAND = 0.05; // radians (~3 degrees)
  void apply_deadband_compensation() {
    for (size_t i = 0; i < 6; ++i) {
      double error = hw_commands_[i] - hw_positions_[i];
      
      // Only apply correction if error is above deadband
      if (std::abs(error) > POSITION_DEADBAND) {
        // Apply small correction in the direction of error
        double correction_sign = (error > 0) ? 1.0 : -1.0;
        corrected_commands_[i] += correction_sign * 0.02; // Small step correction
      }
    }
  } //apply_deadband_compensation



  
}; // class PsyonicHandHardware

}  // namespace psyonic_hardware_interface