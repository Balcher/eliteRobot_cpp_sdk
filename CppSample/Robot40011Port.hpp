﻿#ifndef __ROBOT40011PORT_HPP__
#define __ROBOT40011PORT_HPP__

#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

class Robot40011Port
{
public:

	/// @brief 逆运动学解算
	/// @param poses     位姿[x,y,z,r,p,y] 单位m和rad
	/// @param degrees   六个电机的角度值
	/// @return 是否有解
	bool Robot40011Port::getIKDegrees(const std::vector<double>& poses, std::vector<double>& degrees);

	/// @brief 连接到指定的 IP 和端口
	/// @param ip 目标 IP 地址
	/// @param port 目标端口
	/// @return 如果连接成功返回true，否则返回false
	bool connect(const std::string& ip, int port);

	/// @brief 断开连接
	void disconnect();

	/// @brief 发送脚本
	/// @param script 要发送的脚本内容
	/// @return 发送的字节数
	int sendScript(const std::string& script);

	/// @brief 接收socket数据
	/// @param out_msg 返回的数据
	/// @param size 发送的字节数
	/// @return 是否成功发送
	int socketRecv(void* out_msg, int size);


	/// @brief 检查当前连接状态
	/// @return 如果已连接返回true，否则返回false
	bool isConnect();

	/// @brief 连接状态枚举
	enum ConnectionState { DISCONNECTED = 0, CONNECTED = 1 };

	// 构造函数和析构函数
	Robot40011Port();
	~Robot40011Port();

private:
	struct Socket;                           ///< 前置生命socket结构
	std::mutex socket_instance_mutex;        ///< 套接字实例的互斥锁
	std::unique_ptr<Socket> robot_socket;    ///< 机器人套接字的智能指针

	ConnectionState connection_state;        ///< 连接状态

};

#endif