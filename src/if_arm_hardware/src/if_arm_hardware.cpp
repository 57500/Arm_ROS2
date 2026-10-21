#include "if_arm_hardware/if_arm_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>

// X 固件参数（D）
static constexpr double  kMaxSpeedVal   = 30000.0;   // 0.1rpm 上限 = 3000.0 rpm
static constexpr uint8_t kChecksum      = 0x6B;      // 校验字节
static constexpr uint8_t kFuncEnable    = 0xF3;
static constexpr uint8_t kFuncSpeed     = 0xF6;
static constexpr uint8_t kFuncReadPos   = 0x36;

namespace if_arm_hardware
{
namespace
{
template <typename T>
bool parse_list(const std::string & text ,std::vector<T> & out)
{
    out.clear();
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss,item,','))
    {
        const auto first =item.find_first_not_of(" \t");
        const auto last =item.find_last_not_of(" \t");

        if (first==std::string::npos)
        {
            return false;
        }

        item =item.substr(first,last-first+1);

        try
        {
            if constexpr(std::is_same_v<T,double>)
            {
                out.push_back(std::stod(item));
            }
            else
            {
                out.push_back(static_cast<T>(std::stoi(item)));
            }
        }
        catch(const std::exception& )
        {
            return false;
        }
    }
    return !out.empty();
}
}

constexpr size_t kDof=6;

hardware_interface::CallbackReturn IfArmHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto logger=rclcpp::get_logger("IfArmHardware");

    const size_t n=info.joints.size();

    if (n!=kDof)
    {
        RCLCPP_ERROR(logger,"期望 %zu 个关节，URDF 里给了 %zu 个", kDof, n);
        return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.clear();
    joint_names_.reserve(n);

    for(const auto & joint:info.joints)
    {
        joint_names_.push_back(joint.name);
    }

    const auto & params=info.hardware_parameters;

    auto read_param=[&params](const std::string & key, std::string& value)
    {
        const auto it=params.find(key);
        if(it==params.end())
        {
            return false;
        }
        value=it->second;
        return true;
    };

    std::string text;

    if(read_param("can_interface",text))
    {
        can_interface_=text;
    }

    std::vector<int> ids;
    if (!read_param("motor_ids",text)||!parse_list<int>(text,ids)||ids.size()!=n)
    {
        RCLCPP_ERROR(logger, "参数 motor_ids 缺失或长度不是 %zu", n);
        return hardware_interface::CallbackReturn::ERROR;
    }

    motor_ids_.clear();
    for(const int id :ids)
    {
        motor_ids_.push_back(static_cast<uint8_t>(id));
    }

    if (!read_param("gear_ratios", text) || !parse_list<double>(text, gear_ratios_) ||
    gear_ratios_.size() != n)
    {
        RCLCPP_ERROR(logger, "参数 gear_ratios 缺失或长度不是 %zu", n);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!read_param("directions", text) || !parse_list<int>(text, directions_) ||
        directions_.size() != n)
    {
        RCLCPP_ERROR(logger, "参数 directions 缺失或长度不是 %zu", n);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!read_param("zero_offsets", text) || !parse_list<double>(text, zero_offsets_) ||
        zero_offsets_.size() != n)
    {
        RCLCPP_ERROR(logger, "参数 zero_offsets 缺失或长度不是 %zu", n);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (read_param("max_motor_rpm", text))
    {
    max_motor_rpm_ = std::stod(text);
    }
    if (read_param("default_acc", text))
    {
        default_acc_ = std::stoi(text);
    }

    pulse_per_rev_.assign(n, 3200);

    hw_positions_.assign(n, 0.0);
    hw_velocities_.assign(n, 0.0);
    hw_cmd_vel_.assign(n, 0.0);
    feedback_count_.assign(n, 0);
    feedback_frames_this_cycle_.assign(n, 0);
    feedback_zero_frame_cycles_.assign(n, 0);
    feedback_single_frame_cycles_.assign(n, 0);
    feedback_multi_frame_cycles_.assign(n, 0);
    feedback_max_frames_per_cycle_.assign(n, 0);
    feedback_max_gap_ns_.assign(n, 0);
    feedback_no_update_cycles_.assign(n, 0);
    feedback_max_no_update_cycles_.assign(n, 0);
    feedback_seen_.assign(n, 0);
    feedback_updated_this_cycle_.assign(n, 0);
    last_feedback_time_.assign(n, SteadyClock::time_point{});
    can_tx_ok_.assign(n, 0);
    can_tx_fail_.assign(n, 0);
    last_can_tx_errno_ = 0;

    cycles_ = 0;

    RCLCPP_INFO(
        logger, "on_init 完成：%zu 关节，can=%s，ratio[0]=%.1f，max_rpm=%.0f",
        joint_names_.size(),
        can_interface_.c_str(),
        gear_ratios_[0],
        max_motor_rpm_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IfArmHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!can_open_socket()) { return hardware_interface::CallbackReturn::ERROR; }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IfArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
IfArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_cmd_vel_[i]));
  }
  return interfaces;
}

hardware_interface::CallbackReturn IfArmHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    prev_positions_.assign(joint_names_.size(), 0.0);
    pos_valid_.assign(joint_names_.size(), 0);
    std::fill(feedback_count_.begin(), feedback_count_.end(), 0);
    std::fill(feedback_frames_this_cycle_.begin(), feedback_frames_this_cycle_.end(), 0);
    std::fill(feedback_zero_frame_cycles_.begin(), feedback_zero_frame_cycles_.end(), 0);
    std::fill(feedback_single_frame_cycles_.begin(), feedback_single_frame_cycles_.end(), 0);
    std::fill(feedback_multi_frame_cycles_.begin(), feedback_multi_frame_cycles_.end(), 0);
    std::fill(feedback_max_frames_per_cycle_.begin(), feedback_max_frames_per_cycle_.end(), 0);
    std::fill(feedback_max_gap_ns_.begin(), feedback_max_gap_ns_.end(), 0);
    std::fill(feedback_no_update_cycles_.begin(), feedback_no_update_cycles_.end(), 0);
    std::fill(feedback_max_no_update_cycles_.begin(), feedback_max_no_update_cycles_.end(), 0);
    std::fill(feedback_seen_.begin(), feedback_seen_.end(), 0);
    std::fill(feedback_updated_this_cycle_.begin(),
              feedback_updated_this_cycle_.end(), 0);
    std::fill(last_feedback_time_.begin(), last_feedback_time_.end(),
              SteadyClock::time_point{});

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        send_enable(motor_ids_[i]);                 // 使能（不等待应答）
        hw_cmd_vel_[i] = 0.0;                       // 从静止开始
    }
    RCLCPP_INFO(rclcpp::get_logger("IfArmHardware"), "on_activate：已发使能");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IfArmHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    const double avg_cycle_ms =
    cycle_count_ > 0
        ? static_cast<double>(cycle_period_sum_ns_) /
              static_cast<double>(cycle_count_) / 1e6
        : 0.0;

    const double avg_read_us =
    read_count_ > 0
        ? static_cast<double>(read_time_sum_ns_) /
              static_cast<double>(read_count_) / 1e3
        : 0.0;

    const double avg_write_us =
    write_count_ > 0
        ? static_cast<double>(write_time_sum_ns_) /
              static_cast<double>(write_count_) / 1e3
        : 0.0;

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        send_speed(motor_ids_[i], 0x00, 0,
                   static_cast<uint16_t>(std::clamp(default_acc_, 1, 65535)));   // 立刻发 0 速度，斜率与运行时一致
    }
    if (can_fd_ >= 0)
    {
        ::close(can_fd_);
        can_fd_ = -1;
    }
    uint64_t total_ok = 0;
    uint64_t total_fail = 0;
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        total_ok += can_tx_ok_[i];
        total_fail += can_tx_fail_[i];
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IfArmHardware"),
        "CAN发送统计：ok=%lu fail=%lu last_errno=%d",
        static_cast<unsigned long>(total_ok),
        static_cast<unsigned long>(total_fail),
        last_can_tx_errno_);
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("IfArmHardware"),
            "CAN发送统计：motor=%u ok=%lu fail=%lu",
            static_cast<unsigned>(motor_ids_[i]),
            static_cast<unsigned long>(can_tx_ok_[i]),
            static_cast<unsigned long>(can_tx_fail_[i]));
    }
    RCLCPP_INFO(rclcpp::get_logger("IfArmHardware"), "on_deactivate：已停转并关闭 CAN");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("IfArmHardware"),
            "反馈统计：motor=%u count=%lu max_gap_ms=%.3f max_no_update_cycles=%lu",
            static_cast<unsigned>(motor_ids_[i]),
            static_cast<unsigned long>(feedback_count_[i]),
            static_cast<double>(feedback_max_gap_ns_[i]) / 1e6,
            static_cast<unsigned long>(feedback_max_no_update_cycles_[i]));
        RCLCPP_INFO(
            rclcpp::get_logger("IfArmHardware"),
            "反馈分布：motor=%u sampled_cycles=%lu zero_cycles=%lu single_cycles=%lu "
            "multi_cycles=%lu max_frames_per_cycle=%lu overwritten_frames=%lu",
            static_cast<unsigned>(motor_ids_[i]),
            static_cast<unsigned long>(feedback_zero_frame_cycles_[i] +
                feedback_single_frame_cycles_[i] + feedback_multi_frame_cycles_[i]),
            static_cast<unsigned long>(feedback_zero_frame_cycles_[i]),
            static_cast<unsigned long>(feedback_single_frame_cycles_[i]),
            static_cast<unsigned long>(feedback_multi_frame_cycles_[i]),
            static_cast<unsigned long>(feedback_max_frames_per_cycle_[i]),
            static_cast<unsigned long>(feedback_count_[i] -
                feedback_single_frame_cycles_[i] - feedback_multi_frame_cycles_[i]));
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IfArmHardware"),
        "周期统计：intervals=%lu avg_period_ms=%.6f min_period_ms=%.6f max_period_ms=%.6f "
        "read_count=%lu avg_read_us=%.3f max_read_us=%.3f "
        "write_count=%lu avg_write_us=%.3f max_write_us=%.3f",
        static_cast<unsigned long>(cycle_count_), avg_cycle_ms,
        static_cast<double>(cycle_period_min_ns_) / 1e6,
        static_cast<double>(cycle_period_max_ns_) / 1e6,
        static_cast<unsigned long>(read_count_), avg_read_us,
        static_cast<double>(read_time_max_ns_) / 1e3,
        static_cast<unsigned long>(write_count_), avg_write_us,
        static_cast<double>(write_time_max_ns_) / 1e3);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IfArmHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("IfArmHardware"), "on_error（M4 在这里尽力停转）");
  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::return_type IfArmHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    const auto cycle_start = SteadyClock::now();
    const auto read_start = cycle_start;
    std::fill(feedback_frames_this_cycle_.begin(), feedback_frames_this_cycle_.end(), 0);
    std::fill(feedback_updated_this_cycle_.begin(),
              feedback_updated_this_cycle_.end(), 0);

    if (cycle_clock_valid_) {
        const auto period_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                cycle_start - last_cycle_start_).count();

        ++cycle_count_;
        cycle_period_sum_ns_ += static_cast<uint64_t>(period_ns);

        if (cycle_period_min_ns_ == 0 ||
            static_cast<uint64_t>(period_ns) < cycle_period_min_ns_) {
            cycle_period_min_ns_ = static_cast<uint64_t>(period_ns);
        }

        cycle_period_max_ns_ =
            std::max(cycle_period_max_ns_,
                    static_cast<uint64_t>(period_ns));
    }

    last_cycle_start_ = cycle_start;
    cycle_clock_valid_ = true;

    if (can_fd_ < 0)
    {
        return hardware_interface::return_type::ERROR;
    }

    const double dt = period.seconds();

    // ① 先收上一拍请求的应答
    uint8_t addr[32], func[32], data[32][8];
    const int n = can_drain(addr, func, data, 32);

    for (int k = 0; k < n; ++k)
    {
        if (func[k] != kFuncReadPos || data[k][6] != kChecksum)
        {
            continue;
        }

        for (size_t i = 0; i < motor_ids_.size(); ++i)
        {
            if (motor_ids_[i] != addr[k])
            {
                continue;
            }

            const double pos = reply_to_joint_rad(i, data[k]);
            hw_positions_[i] = pos;

            ++feedback_count_[i];
            ++feedback_frames_this_cycle_[i];
            feedback_updated_this_cycle_[i] = 1;
            if (feedback_seen_[i] != 0)
            {
                const auto gap_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    cycle_start - last_feedback_time_[i]).count();
                feedback_max_gap_ns_[i] = std::max(
                    feedback_max_gap_ns_[i], static_cast<uint64_t>(gap_ns));
            }
            last_feedback_time_[i] = cycle_start;
            feedback_seen_[i] = 1;

            if (pos_valid_[i] && dt > 1e-6)
            {
                const double raw_vel = (pos - prev_positions_[i]) / dt;
                hw_velocities_[i] = 0.2 * raw_vel + 0.8 * hw_velocities_[i];   // 低通，抑制量化噪声
            }
            prev_positions_[i] = pos;
            pos_valid_[i] = 1;
            break;
        }
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        const auto frames = feedback_frames_this_cycle_[i];
        feedback_max_frames_per_cycle_[i] = std::max(
            feedback_max_frames_per_cycle_[i], frames);
        if (frames == 0)
        {
            ++feedback_zero_frame_cycles_[i];
        }
        else if (frames == 1)
        {
            ++feedback_single_frame_cycles_[i];
        }
        else
        {
            ++feedback_multi_frame_cycles_[i];
        }
        if (feedback_updated_this_cycle_[i] != 0)
        {
            feedback_no_update_cycles_[i] = 0;
        }
        else
        {
            ++feedback_no_update_cycles_[i];
            feedback_max_no_update_cycles_[i] = std::max(
                feedback_max_no_update_cycles_[i], feedback_no_update_cycles_[i]);
        }
    }

    // ② 发下一轮的读请求（不等应答）
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        send_read_pos(motor_ids_[i]);

    }

    const auto read_elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(
        SteadyClock::now() - read_start).count();

    ++read_count_;
    read_time_sum_ns_ += static_cast<uint64_t>(read_elapsed);
    read_time_max_ns_ =std::max(read_time_max_ns_, static_cast<uint64_t>(read_elapsed));


    ++cycles_;
    return hardware_interface::return_type::OK;
}



hardware_interface::return_type IfArmHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    const auto write_start = SteadyClock::now();


    if (can_fd_ < 0)
    {
        return hardware_interface::return_type::ERROR;
    }

    // 硬件层最后一道限幅：60 deg/s（D）
    static constexpr double kMaxJointVel = 1.0472;
    for (size_t i = 0; i < hw_cmd_vel_.size(); ++i)
    {
        hw_cmd_vel_[i] = std::clamp(hw_cmd_vel_[i], -kMaxJointVel, kMaxJointVel);
    }

    const uint16_t acc = static_cast<uint16_t>(
        std::max(1, std::min(default_acc_, 65535)));      // acc 不能为 0

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t dir = 0;
        const uint16_t val = joint_rad_s_to_speed_val(i, hw_cmd_vel_[i], &dir);
        send_speed(motor_ids_[i], dir, val, acc);
    }

    const auto write_elapsed =std::chrono::duration_cast<std::chrono::nanoseconds>(
        SteadyClock::now() - write_start).count();

    ++write_count_;
    write_time_sum_ns_ += static_cast<uint64_t>(write_elapsed);
    write_time_max_ns_ =std::max(write_time_max_ns_, static_cast<uint64_t>(write_elapsed));




    return hardware_interface::return_type::OK;
}



// 关节 rad/s → X 固件的速度值(0.1rpm) + 方向位
uint16_t IfArmHardware::joint_rad_s_to_speed_val(size_t i, double omega, uint8_t * dir) const
{
    const double rpm = std::fabs(omega) * 60.0 / (2.0 * M_PI) * gear_ratios_[i];
    double val = rpm * 10.0;                                  // 0.1rpm 单位
    const double max_val = std::min(kMaxSpeedVal, max_motor_rpm_ * 10.0);

    if (val > max_val)
    {
        val = max_val;
    }

    // 死区：电机侧 < 1.0 RPM 的指令发 0。
    // 静止时 JTC 的位置 P 反馈会给出 0.4~0.6 RPM 的小指令，机械上推不动时，
    // 驱动器速度环误差恒为正，电流会持续爬升到上限并触发堵转保护。
    static constexpr double kDeadbandVal = 10.0;              // 0.1rpm 单位
    if (val < kDeadbandVal)
    {
        val = 0.0;
    }

    // 方向位：00=正, 01=负；directions_ 用来把"关节正方向"对齐到"电机正方向"
    const bool positive = (omega >= 0.0) == (directions_[i] > 0);
    *dir = positive ? 0x00 : 0x01;
    return static_cast<uint16_t>(val + 0.5);
}

// 位置应答 → 关节 rad
// data = {0x36, 符号, P3, P2, P1, P0, 0x6B}；X 固件单位 0.1°
double IfArmHardware::reply_to_joint_rad(size_t i, const uint8_t * d) const
{
    const int32_t raw = (static_cast<int32_t>(d[2]) << 24) | (static_cast<int32_t>(d[3]) << 16) |
                        (static_cast<int32_t>(d[4]) << 8)  |  static_cast<int32_t>(d[5]);
    const double deg = ((d[1] == 0x01) ? -1.0 : 1.0) * static_cast<double>(raw) / 10.0;
    return deg * M_PI / 180.0 / gear_ratios_[i] * static_cast<double>(directions_[i])
           + zero_offsets_[i];
}


bool IfArmHardware::can_open_socket()
{
    can_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("IfArmHardware"), "socket: %s", std::strerror(errno));
        return false;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("IfArmHardware"), "ioctl(%s): %s",
                     can_interface_.c_str(), std::strerror(errno));
        ::close(can_fd_); can_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("IfArmHardware"), "bind: %s", std::strerror(errno));
        ::close(can_fd_); can_fd_ = -1;
        return false;
    }

    // 非阻塞：read() 里绝不能等
    const int flags = ::fcntl(can_fd_, F_GETFL, 0);
    ::fcntl(can_fd_, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(rclcpp::get_logger("IfArmHardware"), "CAN %s 已打开", can_interface_.c_str());
    return true;
}

// 逻辑帧分包：payload = [功能码, 数据..., 0x6B]，每包 8 字节
bool IfArmHardware::can_send_logical(uint8_t addr, const uint8_t * payload, size_t len)
{
    size_t pos = 0;
    uint8_t pack = 0;
    do {
        struct can_frame f{};
        f.can_id  = ((static_cast<uint32_t>(addr) << 8) | pack) | CAN_EFF_FLAG;
        size_t n  = len - pos;
        if (n > 8) { n = 8; }
        for (size_t i = 0; i < n; ++i) { f.data[i] = payload[pos + i]; }
        f.can_dlc = static_cast<uint8_t>(n);

        const ssize_t written = ::write(can_fd_, &f, sizeof(f));
        size_t motor_index = motor_ids_.size();
        for (size_t i = 0; i < motor_ids_.size(); ++i)
        {
            if (motor_ids_[i] == addr)
            {
                motor_index = i;
                break;
            }
        }

        if (written != static_cast<ssize_t>(sizeof(f))) {
            if (motor_index < can_tx_fail_.size())
            {
                ++can_tx_fail_[motor_index];
            }
            last_can_tx_errno_ = errno;
            return false;
        }
        if (motor_index < can_tx_ok_.size())
        {
            ++can_tx_ok_[motor_index];
        }
        pos += n;
        ++pack;
    } while (pos < len);
    return true;
}

// 非阻塞收帧，返回收到的条数
int IfArmHardware::can_drain(uint8_t addr_out[], uint8_t func_out[],
                             uint8_t data_out[][8], int max_n)
{
    int n = 0;
    int reads = 0;
    struct can_frame rx{};
    while (reads < max_n &&
           ::read(can_fd_, &rx, sizeof(rx)) == static_cast<ssize_t>(sizeof(rx))) {
        ++reads;
        if ((rx.can_id & CAN_EFF_FLAG) == 0) { continue; }
        const uint32_t id = rx.can_id & CAN_EFF_MASK;
        addr_out[n] = static_cast<uint8_t>((id >> 8) & 0xFF);
        func_out[n] = rx.data[0];
        std::memset(data_out[n], 0, 8);
        std::memcpy(data_out[n], rx.data, rx.can_dlc);
        ++n;
    }
    return n;
}

bool IfArmHardware::send_enable(uint8_t addr)
{
    const uint8_t p[] = {kFuncEnable, 0xAB, 0x01, 0x00, kChecksum};   // 01=使能, 00=立即执行
    return can_send_logical(addr, p, sizeof(p));
}

bool IfArmHardware::send_speed(uint8_t addr, uint8_t dir, uint16_t speed_val, uint16_t acc)
{
    const uint8_t p[] = {
        kFuncSpeed, dir,
        static_cast<uint8_t>(acc >> 8), static_cast<uint8_t>(acc & 0xFF),
        static_cast<uint8_t>(speed_val >> 8), static_cast<uint8_t>(speed_val & 0xFF),
        0x00,                                  // 同步标志：00=立即执行
        kChecksum};
    return can_send_logical(addr, p, sizeof(p));
}

bool IfArmHardware::send_read_pos(uint8_t addr)
{
    const uint8_t p[] = {kFuncReadPos, kChecksum};
    return can_send_logical(addr, p, sizeof(p));
}



}


// ============================================================================
//  PLUGINLIB_EXPORT_CLASS —— 在 .so 里"注册"这个类
//
//  【宏展开后是什么】
//    生成一个全局静态对象 + 一个静态初始化函数。
//    当 pluginlib dlopen 这个 .so 时，该函数自动执行，
//    把 "类名 → 如何 new 出它" 登记到 class_loader 的全局注册表。
//
//  【为什么不写它会失败】
//    编译照样成功（只是 .so 里少一个符号），
//    但运行时 pluginlib 找不到类 → "Could not find class 'if_arm_hardware/IfArmHardware'"。
//    这是最典型的"编译通过、运行起不来"。
//
//  【为什么放在 namespace 外、文件末尾】
//    宏展开成全局符号，放进 namespace 会编译报错；
//    放末尾不打断上面的阅读顺序。
//
//  【两个参数】
//    ① C++ 完整类名（含 namespace）
//    ② 基类（必须与 plugin xml 的 base_class_type 一致）
// ============================================================================
PLUGINLIB_EXPORT_CLASS(if_arm_hardware::IfArmHardware, hardware_interface::SystemInterface)
