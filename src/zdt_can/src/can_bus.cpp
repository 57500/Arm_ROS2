#include "zdt_can/can_bus.hpp"

#include <fcntl.h>              
#include <linux/can.h>         
#include <linux/can/raw.h>  
#include <net/if.h>          
#include <sys/ioctl.h>       
#include <sys/socket.h>       
#include <unistd.h>           

#include <cerrno>
#include <cstring>            
#include <string>

namespace zdt
{
namespace
{
CanResult make_fail(CanError code, const std::string & what)
{
  const int e = errno;
  return {code, e, what + ": " + std::strerror(e)};
}
}  

CanBus::~CanBus() { close(); }

CanResult CanBus::open(const std::string & iface)
{
  close();                 
  iface_ = iface;

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) 
  {
    return make_fail(CanError::SocketFailed, "创建 CAN socket 失败");
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);

  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) 
  {
    auto r = make_fail(CanError::IfaceNotFound,
                       "找不到接口 " + iface + "（是否已执行 ip link set " + iface + " up）");
    close();
    return r;
  }

  struct sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) 
  {
    auto r = make_fail(CanError::BindFailed, "绑定接口 " + iface + " 失败");
    close();
    return r;
  }

  const int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  return {};
}

void CanBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

CanResult CanBus::send(const Frame & f)
{
  if (fd_ < 0) {
    return {CanError::NotOpen, 0, "CAN 未打开"};
  }

  struct can_frame cf{};
  cf.can_id  = f.id | CAN_EFF_FLAG;    
  cf.can_dlc = f.len;

  std::memcpy(cf.data, f.data, f.len);

  const ssize_t n = ::write(fd_, &cf, sizeof(cf));
  if (n != static_cast<ssize_t>(sizeof(cf))) 
  {
    return make_fail(CanError::WriteFailed,
                     "发送失败（电机地址 " + std::to_string(addr_of(f.id)) + "）");
  }

  return {};
}

CanResult CanBus::recv(Frame & out)
{
  if (fd_ < 0) 
  {
    return {CanError::NotOpen, 0, "CAN 未打开"};
  }

  struct can_frame cf{};
  const ssize_t n = ::read(fd_, &cf, sizeof(cf));
  if (n < 0) 
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK) 
    {
      return {CanError::WouldBlock, errno, "当前没有数据"};   
    }

    return make_fail(CanError::ReadFailed, "读取失败");
  }
  if (n != static_cast<ssize_t>(sizeof(cf))) 
  {
    return {CanError::ReadFailed, 0, "读到不完整的 CAN 帧"};
  }

  out.id  = cf.can_id & CAN_EFF_MASK;     
  out.len = cf.can_dlc > 8 ? 8 : cf.can_dlc;
  std::memcpy(out.data, cf.data, out.len);

  return {};
}

} 
