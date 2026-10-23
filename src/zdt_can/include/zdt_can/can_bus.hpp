#ifndef ZDT_CAN__CAN_BUS_HPP_
#define ZDT_CAN__CAN_BUS_HPP_

#include <string>

#include "zdt_can/zdt_protocol.hpp"

namespace zdt
{

// 错误种类。用枚举而不是只返回 bool，调用方才能区分"该重试"和"该报警"。
enum class CanError
{
  Ok,
  NotOpen,        // 还没 open() 就调用了 send / recv
  SocketFailed,   // socket() 失败
  IfaceNotFound,  // 找不到 can0（没插 USB-CAN，或没 ip link set can0 up）
  BindFailed,     // bind() 失败
  WouldBlock,     // 非阻塞读，当前没有数据（正常情况，不是故障）
  WriteFailed,    // 发送失败。err_no=ENOBUFS 表示内核发送队列满（总线太忙）
  ReadFailed      // 读取失败
};

// 一次调用的结果：错误种类 + errno + 可直接写日志的说明
struct CanResult
{
  CanError    code{CanError::Ok};
  int         err_no{0};      // 对应的 errno，0 表示没有
  std::string message;        // 人能看懂的说明
  bool ok() const { return code == CanError::Ok; }
};

// ---------------------------------------------------------------------------
// CanBus：基于 Linux SocketCAN 的收发。只管帧，不认识 ZDT 协议。
// ---------------------------------------------------------------------------
class CanBus
{
public:
  CanBus() = default;
  ~CanBus();                  // 析构时自动 close，避免泄漏文件描述符

  // socket 句柄不能被复制，否则两个对象会各自 close 同一个 fd
  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  CanResult open(const std::string & iface);
  void      close();
  bool      is_open() const { return fd_ >= 0; }

  CanResult send(const Frame & f);
  CanResult recv(Frame & out);     // 非阻塞；没数据返回 WouldBlock

private:
  int         fd_{-1};   // socket 文件描述符，-1 表示未打开
  std::string iface_;    // 接口名，写日志用
};

}  // namespace zdt

#endif  // ZDT_CAN__CAN_BUS_HPP_
