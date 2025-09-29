#ifndef __RTSI_MSG_HPP__
#define __RTSI_MSG_HPP__

#include <stdint.h>

namespace RtsiMsg
{

#pragma pack(1)

// RTSI 报文头
struct Head {
    uint16_t size;
    uint8_t type;
};

// 接收协议检查的报文
struct ProtocolTx {
    Head head;
    uint16_t version;
};

struct ProtocolRx {
    Head head;
    uint8_t is_ok;
};

// 获取版本信息报文
struct ControllerVersionTx{
    Head head;
};

// 回复版本信息
 struct ControllerVersionRx{
    Head head;
    uint32_t major;
    uint32_t minor;
    uint32_t bugfix;
    uint32_t build;
};

// 输出订阅接收的报文
 struct SubscribeOutTx{
    Head head;
    double frequency;
};

// 输入订阅接收报文
struct SubscribeInTx{
    Head head;
};

// 输入输出订阅回复的报文
struct SubscribeRx{
    Head head;
    uint8_t recipe_id;
};

struct DataTxRx {
    Head head;
    uint8_t id;
};

// 开始信号报文
struct StartTx {
    Head head;
};

// 回复开始信号报文
struct StartRx {
    Head head;
    uint8_t is_start;
};

// 开始信号报文
struct PauseTx {
    Head head;
};

// 回复开始信号报文
struct PauseRx {
    Head head;
    uint8_t is_pause;
};

#pragma pack()

} // namespace RtsiMsg







#endif