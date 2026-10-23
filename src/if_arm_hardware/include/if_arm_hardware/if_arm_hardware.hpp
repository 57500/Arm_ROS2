#ifndef IF_ARM_HARDWARE__IF_ARM_HARDWARE_HPP_
#define IF_ARM_HARDWARE__IF_ARM_HARDWARE_HPP_

#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include "zdt_can/zdt_driver.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"


namespace if_arm_hardware
{
class IfArmHardware :public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IfArmHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info
    ) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state
    )override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state
    )override;


    std::vector<hardware_interface::StateInterface> export_state_interfaces()override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;


    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period
    )override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period
    )override;


private:
    using SteadyClock = std::chrono::steady_clock;

    std::string can_interface_{"can0"};
    std::vector<std::string> joint_names_;
    std::vector<uint8_t> motor_ids_;
    std::vector<double> gear_ratios_;
    std::vector<int> directions_;
    std::vector<double> zero_offsets_;
    std::vector<int32_t> pulse_per_rev_;
    double max_motor_rpm_{1000.0};
    int default_acc_{20};


    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_cmd_vel_;

    uint64_t cycles_{0};

    // 关节侧 <-> 电机侧的单位换算（减速比、方向、零位是每台臂不同的参数，所以留在这里）
    double joint_rad_s_to_motor_rpm(size_t i, double rad_s) const;
    double motor_deg_to_joint_rad(size_t i, double motor_deg) const;

    // CAN + ZDT 协议的全部细节都在这个对象里；hardware 只调用"电机语言"的接口
    zdt::ZdtDriver driver_;


    // ── 状态处理 ──────────────────────────────────────────────
    std::vector<double>  prev_positions_;   // 上一拍位置（算速度用）
    std::vector<uint8_t> pos_valid_;        // 是否已经收到过位置（0/1）

    // ── 私有函数 ──────────────────────────────────────────────



    // 计时
    bool cycle_clock_valid_{false};
    SteadyClock::time_point last_cycle_start_{};

    uint64_t cycle_count_{0};
    uint64_t cycle_period_sum_ns_{0};
    uint64_t cycle_period_min_ns_{0};
    uint64_t cycle_period_max_ns_{0};

    uint64_t read_count_{0};
    uint64_t read_time_sum_ns_{0};
    uint64_t read_time_max_ns_{0};

    uint64_t write_count_{0};
    uint64_t write_time_sum_ns_{0};
    uint64_t write_time_max_ns_{0};

    // 反馈新鲜度统计
    std::vector<uint64_t> feedback_count_;
    std::vector<uint64_t> feedback_frames_this_cycle_;
    std::vector<uint64_t> feedback_zero_frame_cycles_;
    std::vector<uint64_t> feedback_single_frame_cycles_;
    std::vector<uint64_t> feedback_multi_frame_cycles_;
    std::vector<uint64_t> feedback_max_frames_per_cycle_;
    std::vector<uint64_t> feedback_max_gap_ns_;
    std::vector<uint64_t> feedback_no_update_cycles_;
    std::vector<uint64_t> feedback_max_no_update_cycles_;
    std::vector<uint8_t> feedback_seen_;
    std::vector<uint8_t> feedback_updated_this_cycle_;
    std::vector<SteadyClock::time_point> last_feedback_time_;




};

}

#endif