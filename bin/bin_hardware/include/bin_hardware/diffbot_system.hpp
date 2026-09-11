#ifndef BIN_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define BIN_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bin_hardware/arduino_comms.hpp"
#include "bin_hardware/wheel.hpp"

namespace bin_hardware
{

class DiffDriveBinHardware : public hardware_interface::SystemInterface
{
    struct Config
    {
        std::string left_wheel_name  = "";
        std::string right_wheel_name = "";
        float loop_rate        = 0.0;
        std::string device     = "";
        int baud_rate          = 0;
        int timeout_ms         = 0;
        int enc_counts_per_rev = 0;
        int pid_p_left = 0, pid_d_left = 0, pid_i_left = 0, pid_o_left = 50;
        int pid_p_right = 0, pid_d_right = 0, pid_i_right = 0, pid_o_right = 50;
        int diag_publish_rate = 0;  // 0 = disabled; N = publish every N read() calls

        std::string cover_joint_name = "bin_cover_joint";
        double cover_closed_rad = 0.0;
        double cover_open_rad   = -M_PI / 2.0;
    };

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveBinHardware);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    ArduinoComms comms_;
    Config cfg_;

    Wheel wheel_l_;
    Wheel wheel_r_;

    double cover_cmd_ = std::numeric_limits<double>::quiet_NaN();
    double cover_pos_ = 0.0;
    double cover_vel_ = 0.0;
    int last_lid_state_ = -1;  // -1 = unsent, 0 = closed, 1 = open

    rclcpp::Node::SharedPtr diag_node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;
    int diag_cycle_count_ = 0;
};

}  // namespace bin_hardware

#endif  // BIN_HARDWARE__DIFFBOT_SYSTEM_HPP_
