#ifndef ZDT_CAN_ZDT_PROTOCOL_HPP
#define ZDT_CAN_ZDT_PROTOCOL_HPP

#include <algorithm>
#include <utility>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

namespace zdt
{
constexpr uint8_t kChecksum=0x6B;
constexpr uint8_t kFuncEnable=0xF3;
constexpr uint8_t kFuncSpeed=0xF6;
constexpr uint8_t kFuncReadPos=0x36;

struct Frame
{
    uint32_t id{0};
    uint8_t len{0};
    uint8_t data[8]{0};
};

inline uint8_t addr_of(uint32_t id)
{
    return static_cast<uint8_t>((id>>8)&0xFF);
}

inline uint8_t packet_of(uint32_t id)
{
    return static_cast<uint8_t>(id&0xFF);
}

inline std::vector<Frame> split_packets(uint8_t addr,const uint8_t* payload,size_t len)
{
    std::vector<Frame> out;

    if(payload==nullptr||len==0)
    {
        return out;
    }

    const uint8_t func=payload[0];
    size_t pos=0;
    uint8_t pack=0;

    while(pos<len)
    {
        Frame f;
        f.id=(static_cast<uint32_t>(addr)<<8)|pack;

        if(pack==0)
        {
            const size_t n=std::min<size_t>(8,len-pos);

            for(size_t i=0;i<n;++i)
            {
                f.data[i]=payload[pos+i];
            }
            f.len=static_cast<uint8_t>(n);

            pos+=n;
        }
        else
        {
            const size_t n=std::min<size_t>(7,len-pos);
            f.data[0]=func;
            for(size_t i=0;i<n;++i)
            {
                f.data[1+i]=payload[pos+i];
            }
            f.len=static_cast<uint8_t>(n+1);

            pos+=n;
        }

        ++pack;
        out.push_back(f);
    }

    return out;
}

inline std::vector<uint8_t> encode_enable(bool on)
{
    return {kFuncEnable,0xAB,static_cast<uint8_t>(on?1:0),0x00,kChecksum};
}

inline std::vector<uint8_t> encode_speed(uint8_t dir,uint16_t acc,uint16_t speed_val)
{
    return {kFuncSpeed,dir,static_cast<uint8_t>(acc>>8),static_cast<uint8_t>(acc & 0xFF),
        static_cast<uint8_t>(speed_val>>8),static_cast<uint8_t>(speed_val & 0xFF),0x00,kChecksum};
}

inline std::vector<uint8_t> encode_read_pos()
{
  return {kFuncReadPos, kChecksum};
}

inline std::vector<uint8_t> encode_auto_report(uint8_t func, uint16_t period_ms)
{
  return {0x11, 0x18, func,static_cast<uint8_t>(period_ms >> 8), 
    static_cast<uint8_t>(period_ms & 0xFF),kChecksum};
}

inline std::optional<double> decode_position_deg(const uint8_t * d, size_t len)
{
  if (d == nullptr || len < 7 || d[0] != kFuncReadPos || d[6] != kChecksum) 
  {
    return std::nullopt;
  }

  const uint32_t raw = (static_cast<uint32_t>(d[2]) << 24) |
                       (static_cast<uint32_t>(d[3]) << 16) |
                       (static_cast<uint32_t>(d[4]) << 8)  |
                        static_cast<uint32_t>(d[5]);

  const double sign = (d[1] == 0x01) ? -1.0 : 1.0;

  return sign * static_cast<double>(raw) / 10.0;
}


struct Message
{
  uint8_t addr{0};               
  std::vector<uint8_t> bytes;  
};

class Reassembler
{
public:
  std::optional<Message> feed(const Frame & f)
  {
    const uint8_t a= addr_of(f.id);
    auto &buf = pending_[a];           

    if (packet_of(f.id) == 0) 
    {
      buf.assign(f.data, f.data + f.len);      
    } 
    else 
    {
      if (buf.empty() || f.len < 1)
      {
        pending_.erase(a);                    
        return std::nullopt;
      }
      
      buf.insert(buf.end(), f.data + 1, f.data + f.len);
    }

    const bool done = (f.len < 8) || (f.len == 8 && f.data[7] == kChecksum);
    if (!done) 
    {
      return std::nullopt;                  
    }

    Message m{a, std::move(buf)};               
    pending_.erase(a);                        
    return m;
  }

private:
  std::map<uint8_t, std::vector<uint8_t>> pending_; 
};


}




#endif