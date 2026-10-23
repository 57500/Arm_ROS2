#include "zdt_can/zdt_driver.hpp"

#include <algorithm>
#include <cmath>

namespace zdt
{

void ZdtDriver::set_max_rpm(double rpm)
{
  // 夹在 [0, 硬上限] 之间:参数文件写错(负数、超大)也不会突破硬上限
  max_rpm_ = std::clamp(rpm, 0.0, kHardMaxRpm);
}

CanResult ZdtDriver::enable(uint8_t addr, bool on)
{
  return send_payload(addr, encode_enable(on));
}

CanResult ZdtDriver::set_speed(uint8_t addr, double rpm, uint16_t acc)
{
  // 方向用符号位判断,不用 rpm < 0:这样 -0.0 也算负,
  // 和旧代码"零速度时仍按关节方向发方向位"的行为保持一致。
  const uint8_t dir = std::signbit(rpm) ? 0x01 : 0x00;   // 00 = 正转, 01 = 反转

  // 协议速度单位是 0.1 rpm,所以乘 10。NaN / 无穷大一律当 0,避免转成整数时出现未定义行为。
  double val = std::isfinite(rpm) ? std::fabs(rpm) * 10.0 : 0.0;

  val = std::min(val, max_rpm_ * 10.0);       // 截断到上限
  if (val < kDeadbandRpm * 10.0) {            // 死区
    val = 0.0;
  }

  const uint16_t speed_val = static_cast<uint16_t>(val + 0.5);   // 四舍五入

  // 加速度档位的合法范围是协议细节，统一在这里夹住，调用方不用关心
  const uint16_t acc_val = std::clamp<uint16_t>(acc, kAccMin, kAccMax);
  return send_payload(addr, encode_speed(dir, acc_val, speed_val));
}

CanResult ZdtDriver::request_position(uint8_t addr)
{
  return send_payload(addr, encode_read_pos());
}

CanResult ZdtDriver::set_position_report(uint8_t addr, uint16_t period_ms)
{
  // 11 18 36 <period 2B> 6B；period = 0 表示停止上报
  return send_payload(addr, encode_auto_report(kFuncReadPos, period_ms));
}

std::vector<PositionReply> ZdtDriver::poll_positions(int max_frames)
{
  std::vector<PositionReply> out;
  Frame f;
  int n = 0;

  // recv 成功才继续;没数据时返回 WouldBlock,循环自然结束
  while (n < max_frames && bus_.recv(f).ok()) {
    ++n;

    // 拼包:单包消息立即返回,多包消息等最后一包到了才返回
    const std::optional<Message> msg = reassembler_.feed(f);
    if (!msg) {
      continue;
    }

    // 解码:不是位置应答 / 校验码不对 / 长度不够 -> nullopt,直接跳过
    const std::optional<double> deg =
      decode_position_deg(msg->bytes.data(), msg->bytes.size());
    if (deg) {
      out.push_back(PositionReply{msg->addr, *deg});
    }
  }
  return out;
}

TxStats ZdtDriver::tx_stats(uint8_t addr) const
{
  const auto it = stats_.find(addr);
  return it == stats_.end() ? TxStats{} : it->second;
}

CanResult ZdtDriver::send_payload(uint8_t addr, const std::vector<uint8_t> & payload)
{
  TxStats & st = stats_[addr];

  // 一条逻辑命令可能分成多帧(目前用到的命令都是单帧)
  for (const Frame & f : split_packets(addr, payload.data(), payload.size())) {
    const CanResult r = bus_.send(f);
    if (!r.ok()) {
      ++st.fail;
      last_errno_ = r.err_no;
      return r;                 // 一帧失败,后面的就不发了
    }
    ++st.ok;
  }
  return CanResult{};
}

}  // namespace zdt
