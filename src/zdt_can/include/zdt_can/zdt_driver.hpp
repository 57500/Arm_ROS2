#ifndef ZDT_CAN__ZDT_DRIVER_HPP_
#define ZDT_CAN__ZDT_DRIVER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "zdt_can/can_bus.hpp"
#include "zdt_can/zdt_protocol.hpp"

namespace zdt
{

// 一条解码后的位置反馈：哪个电机、电机轴当前角度（单位：度，带符号）
struct PositionReply
{
  uint8_t addr{0};
  double  deg{0.0};
};

// 发送统计（按电机地址分开记）
struct TxStats
{
  uint64_t ok{0};     // 成功发出的帧数
  uint64_t fail{0};   // 发送失败的帧数
};

// ---------------------------------------------------------------------------
// ZdtDriver：对外只讲"电机语言"的一层。
//
//   调用方（hardware）传：电机地址、转速 rpm、使能开关
//   调用方拿到：电机地址 + 角度（度）
//   字节、帧、分包、校验码、功能码、0.1rpm 缩放……全部藏在内部。
//
//   内部组合：CanBus（收发帧）+ zdt_protocol（编解码）+ Reassembler（拼包）
//
//   线程：非线程安全。ros2_control 的 read/write 在同一线程里调用，不需要加锁。
// ---------------------------------------------------------------------------
class ZdtDriver
{
public:
  CanResult open(const std::string & iface) { return bus_.open(iface); }
  void      close() { bus_.close(); }
  bool      is_open() const { return bus_.is_open(); }

  // 运行时转速上限（rpm，电机轴侧）。只能调低，不能超过硬上限 kHardMaxRpm。
  void set_max_rpm(double rpm);

  // ---- 命令 --------------------------------------------------------------
  CanResult enable(uint8_t addr, bool on);

  // 速度模式。rpm 是电机轴转速，带符号：正 = 正转，负 = 反转。
  //   - 超过上限会被截断到上限
  //   - 绝对值小于 kDeadbandRpm 的指令按 0 发（避免堵转时电流爬升，见 j2 堵转问题）
  //   - acc 是协议里的加速度档位（0~255 有效），不是物理单位
  CanResult set_speed(uint8_t addr, double rpm, uint16_t acc);

  // 请求一次位置。应答之后由 poll_positions() 取出
  CanResult request_position(uint8_t addr);

  // 定时上报位置：period_ms = 0 停止；>0 表示每 period_ms 毫秒自动上报一次。
  // 开了之后就不需要每拍 request_position() 了。
  CanResult set_position_report(uint8_t addr, uint16_t period_ms);

  // ---- 反馈 --------------------------------------------------------------
  // 非阻塞：把当前已到达的帧读完（最多 max_frames 帧），拼包、校验、解码，
  // 只返回位置反馈。其他应答（使能回执等）在这里被丢弃。
  std::vector<PositionReply> poll_positions(int max_frames);

  // ---- 统计 --------------------------------------------------------------
  TxStats tx_stats(uint8_t addr) const;
  int     last_errno() const { return last_errno_; }   // 最近一次发送失败的 errno

  // ---- 常量 --------------------------------------------------------------
  static constexpr double   kHardMaxRpm  = 3000.0;   // 固件速度上限，写死，不可配置
  static constexpr double   kDeadbandRpm = 1.0;      // 死区
  static constexpr uint16_t kAccMin      = 1;        // 加速度档位下限：0 会被电机拒绝
  static constexpr uint16_t kAccMax      = 65535;    // 加速度档位是协议里的 2 字节

private:
  CanResult send_payload(uint8_t addr, const std::vector<uint8_t> & payload);

  CanBus                        bus_;          // 直接用它收发，不再套一层抽象
  Reassembler                   reassembler_;
  double                        max_rpm_{kHardMaxRpm};
  std::map<uint8_t, TxStats>    stats_;
  int                           last_errno_{0};
};

}  // namespace zdt

#endif  // ZDT_CAN__ZDT_DRIVER_HPP_
