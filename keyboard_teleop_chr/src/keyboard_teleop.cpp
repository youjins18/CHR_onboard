#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iostream>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher()
    : Node("keyboard_publisher"), stop_flag_(false)
    {
        j1_swing_amplitude_ = static_cast<float>(
            this->declare_parameter<double>("j1_swing_amplitude_deg", 90.0) * kPi / 180.0);
        j1_swing_frequency_ =
            this->declare_parameter<double>("j1_swing_frequency_hz", 0.05);

        flags_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "control_mode_flags", 10);
        j1_command_pub_ =
            this->create_publisher<std_msgs::msg::Float32>("j1_angle_cmd", 10);
        j1_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "j1_angle", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(flags_mutex_);
                j1_measured_ = msg->data;
                j1_feedback_received_ = true;
            });

        // [DOB, custom mode, legacy trajectory, H mode, L1,
        //  mode 1, mode 2, mode 3, measured-pose hold]
        flags_ = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        mode_start_ = std::chrono::steady_clock::now();

        command_timer_ = this->create_wall_timer(10ms, [this]() {
            std::lock_guard<std::mutex> lock(flags_mutex_);
            publish_flags_locked();
            publish_active_j1_locked();
        });

        print_help();
        input_thread_ = std::thread(&KeyboardPublisher::keyboard_loop, this);
    }

    ~KeyboardPublisher()
    {
        stop_flag_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr flags_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr j1_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr j1_angle_sub_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::thread input_thread_;
    std::atomic<bool> stop_flag_;
    std::vector<int32_t> flags_;
    std::mutex flags_mutex_;
    std::chrono::steady_clock::time_point mode_start_;
    float j1_swing_amplitude_{static_cast<float>(kPi / 2.0)};
    double j1_swing_frequency_{0.05};
    float j1_measured_{0.0f};
    float j1_hold_command_{0.0f};
    float j1_last_command_{0.0f};
    bool j1_feedback_received_{false};

    static void print_help()
    {
        std::cout << R"(
===============================================================
                    CHR KEYBOARD CONTROL
===============================================================

       d                       g                       h
  (DOB ON/OFF)       (Palletrone trajectory)   (J1 0-90 deg + arc)

       1                       2                       3
 (J1 0 deg + Z)        (J1 45 deg + XZ)         (J1 90 deg + X)

                            SPACE
                    (hold measured pose)

  q : quit
===============================================================
)" << std::flush;
    }

    void publish_flags_locked()
    {
        std_msgs::msg::Int32MultiArray msg;
        msg.data = flags_;
        flags_pub_->publish(msg);
    }

    void publish_j1_locked(float command)
    {
        std_msgs::msg::Float32 msg;
        msg.data = command;
        j1_command_pub_->publish(msg);
        j1_last_command_ = command;
    }

    void publish_active_j1_locked()
    {
        if (flags_[3]) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - mode_start_).count();
            publish_j1_locked(0.5f * j1_swing_amplitude_ * static_cast<float>(
                1.0 - std::cos(2.0 * kPi * j1_swing_frequency_ * elapsed)));
        } else if (flags_[5]) {
            publish_j1_locked(0.0f);
        } else if (flags_[6]) {
            publish_j1_locked(static_cast<float>(kPi / 4.0));
        } else if (flags_[7]) {
            publish_j1_locked(static_cast<float>(kPi / 2.0));
        } else if (flags_[8]) {
            publish_j1_locked(j1_hold_command_);
        }
    }

    void clear_motion_flags_locked()
    {
        flags_[1] = 0;
        flags_[2] = 0;
        flags_[3] = 0;
        flags_[5] = 0;
        flags_[6] = 0;
        flags_[7] = 0;
        flags_[8] = 0;
    }

    void toggle_motion_mode_locked(size_t flag_index)
    {
        const bool enable = flags_[flag_index] == 0;
        clear_motion_flags_locked();

        if (enable) {
            flags_[1] = 1;
            flags_[flag_index] = 1;
            mode_start_ = std::chrono::steady_clock::now();
        }

        publish_flags_locked();
        publish_active_j1_locked();
    }

    void hold_measured_pose_locked()
    {
        clear_motion_flags_locked();
        flags_[1] = 1;
        flags_[8] = 1;
        j1_hold_command_ = j1_feedback_received_ ? j1_measured_ : j1_last_command_;
        publish_flags_locked();
        publish_active_j1_locked();
    }

    void keyboard_loop()
    {
        struct termios oldt, newt;
        char ch;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (!stop_flag_ && rclcpp::ok()) {
            ch = getchar();

            if (ch == '\x1b') {
                getchar();
                getchar();
                continue;
            }

            std::lock_guard<std::mutex> lock(flags_mutex_);
            const char key =
                static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

            switch (key) {
                case 'd':
                    flags_[0] = !flags_[0];
                    publish_flags_locked();
                    break;

                case 'g':
                    toggle_motion_mode_locked(2);
                    break;

                case 'h':
                    toggle_motion_mode_locked(3);
                    break;

                case '1':
                    toggle_motion_mode_locked(5);
                    break;

                case '2':
                    toggle_motion_mode_locked(6);
                    break;

                case '3':
                    toggle_motion_mode_locked(7);
                    break;

                case ' ':
                    hold_measured_pose_locked();
                    break;

                case 'q':
                    std::fill(flags_.begin(), flags_.end(), 0);
                    publish_flags_locked();
                    rclcpp::shutdown();
                    break;

                default:
                    break;
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
