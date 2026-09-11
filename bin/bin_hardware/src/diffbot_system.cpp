#include "bin_hardware/diffbot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "std_msgs/msg/float32_multi_array.hpp"

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bin_hardware
{

hardware_interface::CallbackReturn DiffDriveBinHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name  = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate        = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
    cfg_.device           = info_.hardware_parameters["device"];
    cfg_.baud_rate        = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms       = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    if (info_.hardware_parameters.count("diag_publish_rate") > 0)
        cfg_.diag_publish_rate = std::stoi(info_.hardware_parameters["diag_publish_rate"]);

    if (info_.hardware_parameters.count("pid_p_left") > 0)
    {
        cfg_.pid_p_left  = std::stoi(info_.hardware_parameters["pid_p_left"]);
        cfg_.pid_d_left  = std::stoi(info_.hardware_parameters["pid_d_left"]);
        cfg_.pid_i_left  = std::stoi(info_.hardware_parameters["pid_i_left"]);
        cfg_.pid_o_left  = std::stoi(info_.hardware_parameters["pid_o_left"]);
        cfg_.pid_p_right = std::stoi(info_.hardware_parameters["pid_p_right"]);
        cfg_.pid_d_right = std::stoi(info_.hardware_parameters["pid_d_right"]);
        cfg_.pid_i_right = std::stoi(info_.hardware_parameters["pid_i_right"]);
        cfg_.pid_o_right = std::stoi(info_.hardware_parameters["pid_o_right"]);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "PID values not supplied, using firmware defaults.");
    }

    if (info_.hardware_parameters.count("cover_joint_name") > 0)
        cfg_.cover_joint_name = info_.hardware_parameters["cover_joint_name"];
    if (info_.hardware_parameters.count("cover_closed_rad") > 0)
        cfg_.cover_closed_rad = hardware_interface::stod(info_.hardware_parameters["cover_closed_rad"]);
    if (info_.hardware_parameters.count("cover_open_rad") > 0)
        cfg_.cover_open_rad = hardware_interface::stod(info_.hardware_parameters["cover_open_rad"]);

    wheel_l_.setup(cfg_.left_wheel_name,  cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    RCLCPP_INFO(get_logger(),
        "rads_per_count: l=%.6f r=%.6f",
        wheel_l_.rads_per_count, wheel_r_.rads_per_count);

    // Validate joints
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.name == cfg_.cover_joint_name)
        {
            if (joint.command_interfaces.size() != 1 ||
                joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(get_logger(),
                    "Cover joint '%s' must have exactly 1 '%s' command interface.",
                    joint.name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.empty() ||
                joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(get_logger(),
                    "Cover joint '%s' must have '%s' as first state interface.",
                    joint.name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            continue;
        }

        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has %zu command interfaces. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' command interface. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has %zu state interfaces. 2 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveBinHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
    state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel);

    state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos);
    state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel);

    state_interfaces.emplace_back(cfg_.cover_joint_name, hardware_interface::HW_IF_POSITION, &cover_pos_);
    state_interfaces.emplace_back(cfg_.cover_joint_name, hardware_interface::HW_IF_VELOCITY, &cover_vel_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveBinHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd);
    command_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd);

    command_interfaces.emplace_back(cfg_.cover_joint_name, hardware_interface::HW_IF_POSITION, &cover_cmd_);

    return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveBinHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    if (comms_.connected()) comms_.disconnect();
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    if (cfg_.diag_publish_rate > 0)
    {
        // Publishers don't need the node to be spun — fire-and-forget publish works
        diag_node_ = rclcpp::Node::make_shared("bin_hw_diag");
        diag_pub_  = diag_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bin/diagnostics", 10);
        RCLCPP_INFO(get_logger(),
            "Diagnostics publishing on /bin/diagnostics every %d read() cycles",
            cfg_.diag_publish_rate);
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBinHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
    if (comms_.connected()) comms_.disconnect();
    diag_pub_.reset();
    diag_node_.reset();
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBinHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    if (!comms_.connected())
    {
        RCLCPP_ERROR(get_logger(), "Cannot activate: serial not connected.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (cfg_.pid_p_left > 0 || cfg_.pid_p_right > 0)
    {
        RCLCPP_INFO(get_logger(),
            "Sending PID gains — left: Kp=%d Kd=%d Ki=%d Ko=%d | right: Kp=%d Kd=%d Ki=%d Ko=%d",
            cfg_.pid_p_left,  cfg_.pid_d_left,  cfg_.pid_i_left,  cfg_.pid_o_left,
            cfg_.pid_p_right, cfg_.pid_d_right, cfg_.pid_i_right, cfg_.pid_o_right);
        comms_.set_pid_values(
            cfg_.pid_p_left,  cfg_.pid_d_left,  cfg_.pid_i_left,  cfg_.pid_o_left,
            cfg_.pid_p_right, cfg_.pid_d_right, cfg_.pid_i_right, cfg_.pid_o_right);
        RCLCPP_INFO(get_logger(), "PID gains sent successfully.");
    }
    else
    {
        RCLCPP_WARN(get_logger(), "PID gains NOT sent — pid_p_left=%d pid_p_right=%d (both must be > 0)",
            cfg_.pid_p_left, cfg_.pid_p_right);
    }

    // Firmware's initServo() already closes the lid at boot; sync reported state to match.
    comms_.set_lid(false);
    last_lid_state_ = 0;
    cover_pos_ = cfg_.cover_closed_rad;

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBinHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    if (comms_.connected())
    {
        comms_.set_motor_values(0, 0);
    }
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveBinHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!comms_.connected()) return hardware_interface::return_type::ERROR;

    if (!comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc))
    {
        // Dropped/garbled frame. Hold the last known position: diff_drive_controller
        // integrates position deltas, so anything else shows up as a teleport in odom.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "encoder read failed; holding last position");
        wheel_l_.vel = 0.0;
        wheel_r_.vel = 0.0;
        return hardware_interface::return_type::OK;
    }

    RCLCPP_DEBUG(get_logger(),
        "ENC l=%d r=%d", wheel_l_.enc, wheel_r_.enc);

    if (cfg_.diag_publish_rate > 0 && diag_pub_)
    {
        if (++diag_cycle_count_ >= cfg_.diag_publish_rate)
        {
            diag_cycle_count_ = 0;
            auto snap = comms_.read_diagnostics();
            if (snap.valid)
            {
                std_msgs::msg::Float32MultiArray msg;
                msg.data = {
                    static_cast<float>(snap.l_enc),
                    static_cast<float>(snap.r_enc),
                    snap.l_tgt_rpm, snap.r_tgt_rpm,
                    snap.l_rpm,     snap.r_rpm,
                    snap.l_err,     snap.r_err,
                    snap.l_int,     snap.r_int,
                    static_cast<float>(snap.l_out),
                    static_cast<float>(snap.r_out),
                    snap.lin_vel,   snap.ang_vel
                };
                diag_pub_->publish(msg);
            }
        }
    }

    double delta_seconds = period.seconds();
    if (delta_seconds == 0.0)
    {
        RCLCPP_WARN_ONCE(get_logger(), "read() called with zero period; skipping velocity update.");
        return hardware_interface::return_type::OK;
    }

    auto update_wheel = [&](Wheel & w) {
        double pos_prev = w.pos;
        w.pos = w.calc_enc_angle();
        w.vel = (w.pos - pos_prev) / delta_seconds;
    };

    update_wheel(wheel_l_);
    update_wheel(wheel_r_);

    // No lid encoder — open-loop echo of the last commanded position.
    if (std::isfinite(cover_cmd_))
    {
        cover_pos_ = cover_cmd_;
    }
    cover_vel_ = 0.0;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveBinHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!comms_.connected()) return hardware_interface::return_type::ERROR;

    int l = static_cast<int>(wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate);
    int r = static_cast<int>(wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "CMD rad/s: l=%.3f r=%.3f  →  counts: l=%d r=%d",
        wheel_l_.cmd, wheel_r_.cmd, l, r);

    comms_.set_motor_values(l, r);

    // Lid: firmware only supports open/closed (SERVO_CMD 's 1'/'s 0'), so send-on-change
    // at the 50% threshold. isfinite() guards against the NaN the position controller
    // holds before its first command arrives.
    if (std::isfinite(cover_cmd_))
    {
        double span = cfg_.cover_open_rad - cfg_.cover_closed_rad;
        double frac = std::clamp((cover_cmd_ - cfg_.cover_closed_rad) / span, 0.0, 1.0);
        int open = (frac >= 0.5) ? 1 : 0;
        if (open != last_lid_state_)
        {
            comms_.set_lid(open == 1);
            last_lid_state_ = open;
        }
    }

    return hardware_interface::return_type::OK;
}

}  // namespace bin_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    bin_hardware::DiffDriveBinHardware, hardware_interface::SystemInterface)
