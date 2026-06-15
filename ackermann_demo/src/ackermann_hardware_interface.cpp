#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>

namespace ackermann_demo {

// Structure for compact serial data packets
struct ControlPacket {
    float left_wheel_vel;
    float right_wheel_vel;
    float left_steer_pos;
    float right_steer_pos;
};

class AckermannHardwareInterface : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Allocate vector buffers to track joints
        hw_commands_.resize(info_.joints.size(), 0.0);
        hw_states_position_.resize(info_.joints.size(), 0.0);
        hw_states_velocity_.resize(info_.joints.size(), 0.0);

        serial_port_name_ = info_.hardware_parameters.at("serial_port");
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto & command_interface_name = info_.joints[i].command_interfaces[0].name;
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, command_interface_name, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
        RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareInterface"), "Connecting to serial port: %s", serial_port_name_.c_str());
        
        // Open the serial device connection line
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("AckermannHardwareInterface"), "Failed to open serial port!");
            return CallbackReturn::ERROR;
        }

        // Configure the serial port hardware settings (115200 Baud, 8N1 raw mode)
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            return CallbackReturn::ERROR;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &tty);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        // Fallback positioning states logic for joints without encoders (like the steering servo)
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
                hw_states_position_[i] = hw_commands_[i];
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        ControlPacket packet;
        
        // Map commands to packet channels sequentially
        packet.left_wheel_vel  = static_cast<float>(hw_commands_[0]);
        packet.right_wheel_vel = static_cast<float>(hw_commands_[1]);
        packet.left_steer_pos  = static_cast<float>(hw_commands_[2]);
        packet.right_steer_pos = static_cast<float>(hw_commands_[3]);

        // Send the packet over the serial bus; log if the write fails.
        if (serial_fd_ >= 0) {
            ssize_t bytes_written = ::write(serial_fd_, &packet, sizeof(packet));
            if (bytes_written < 0) {
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("AckermannHardwareInterface"), 
                    *rclcpp::Clock::make_shared(), 1000, "Serial bus write failed!");
            }
        }

        return hardware_interface::return_type::OK;
    }

private:
    std::string serial_port_name_;
    int serial_fd_ = -1;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
};

} // namespace ackermann_demo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ackermann_demo::AckermannHardwareInterface, hardware_interface::SystemInterface)