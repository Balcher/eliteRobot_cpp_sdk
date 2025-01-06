#ifndef __PRIMARY_PORT_HPP__
#define __PRIMARY_PORT_HPP__

#include "RobotException.hpp"

// 标准库文件
#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <functional>

// 常量定义
#define ITEM_MAX_DATA_BUFF (8)

class RobotState {
    public:
    /// @brief Item结构体定义
    struct Item {
        std::string name;                           ///< 项目的名称
        int size;                                   ///< 项目的大小
        char data_buff[ITEM_MAX_DATA_BUFF];         ///< 数据缓冲区
    };
    /// @brief 连接状态枚举
    enum ConnectionState { DISCONNECTED = 0, CONNECTED = 1 };

    /// @brief 设置异常回调函数
    /// @param callback 处理异常的回调函数
    void setExceptionCallback(const std::function<void(const RobotException&)>& callback);

    /// @brief 加载配置文件
    /// @param file_path 配置文件
    /// @return 如果成功加载返回 true，否则返回 false
    bool loadConfigure(const std::string& file_path);

    /// @brief 连接到指定的 IP 和端口
    /// @param ip 目标 IP 地址
    /// @param port 目标端口
    /// @return 如果连接成功返回true，否则返回false
    bool connect(const std::string& ip, int port);

    /// @brief 检查当前连接状态
    /// @return 如果已连接返回true，否则返回false
    bool isConnect();

    /// @brief 断开连接
    void disconnect();

    /// @brief 发送脚本
    /// @param script 要发送的脚本内容
    /// @return 发送的字节数
    int sendScript(const std::string& script);

    /// @brief 查找指定名称的项目
    /// @param name 项目的名称
    /// @param item 输出的项目指针
    /// @return 如果找到返回 true，否则返回false
    bool findItem(const std::string& name, Item* item);

    /// @brief 获取项目数据
    /// @tparam T 数据类型
    /// @param name 项目的名称
    /// @param data 输出的数据指针
    /// @return 返回获取的数据
    template <typename T>
    T getItemData(const std::string& name, T* data) {
        Item item;
        if (!findItem(name, &item)) {
            return (T)0;
        }
        T result = (T)0;
        if (item.size == sizeof(T)) {
            memcpy(&result, item.data_buff, sizeof(T));
            if (data) {
                *data = result;
            }
        }
        return result;
    }

    // 构造函数和析构函数
    RobotState();
    ~RobotState();

    // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
    RobotState(const RobotState&) = delete;
    RobotState& operator=(const RobotState&) = delete;
    RobotState(RobotState&&) = delete;
    RobotState& operator=(RobotState&&) = delete;

   private:
    class LoadFile; ///< 前置声明 LoadFile类

    std::unique_ptr<std::thread> recv_thread_handle;                     ///< 接收线程句柄
    bool recv_thread_keep_alive;                                         ///< 接收线程是否保持活动

    // 线程安全的锁
    typedef std::unique_lock<std::shared_timed_mutex> WriteLock;         ///< 写锁类型
    typedef std::shared_lock<std::shared_timed_mutex> ReadLock;          ///< 读锁类型
    std::shared_timed_mutex packages_mutex;                              ///< 包的互斥锁
    typedef std::unordered_map<std::string, Item> Packages;              ///< 项目包
    typedef std::vector<std::string> PackagesOrder;                      ///< 项目顺序
    Packages packages;                                                   ///< 存储项目的哈希表
    PackagesOrder packages_order;                                        ///< 存储项目顺序的向量
    std::function<void(const RobotException&)> robot_exception_callback; ///< 异常处理回调函数

    std::mutex socket_instance_mutex;                                    ///< 套接字实例的互斥锁
    struct Sokcet;                                                       ///< 前置声明Socket结构
    std::unique_ptr<Sokcet> robot_socket;                                ///< 机器人套接字的智能指针

    ConnectionState connection_state;                                    ///< 连接状态

    static void flipBytes(void* data, int size);                         ///< 字节翻转
    void updateRobotState(uint8_t* recv_buff);                           ///< 更新机器人状态
    int socketRecv(void* out_msg, int size);                             ///< 接收数据
    bool abortMessageRecv(int size);                                     ///< 中止消息接收
    void recv_thread();                                                  ///< 接收线程的实现
    bool convertByteStream(const uint8_t* bytes, uint32_t bytes_len);    ///< 转换字节流
};

/// @brief LoadFile类定义
class RobotState::LoadFile {
   private:

    /// @brief 分析项目的静态方法
    /// @param line 输入的行字符串，包含项目信息
    /// @param item 输出的项目指针，用于存储解析后的项目
    /// @param prefix 前缀字符串，用于标识项目的类型或分类
    /// @return 如果分析成功返回 true，否则返回 false
    static bool analyzeItem(const std::string& line, RobotState::Item* item, std::string& prefix);

   public:
    /// @brief 从文件加载项目的静态方法
    /// @param file_path 配置文件的路径
    /// @param packages 指向项目包的指针，用于存储加载的项目
    /// @param packages_order 指向项目顺序的指针，用于存储项目的加载顺序
    /// @return 如果加载成功返回 true，否则返回 false
    static bool loadFile(const std::string& file_path, RobotState::Packages* packages, RobotState::PackagesOrder* packages_order);
    
    /// @brief 插入项目的静态方法
    /// @param item 要插入的项目
    /// @param packages 指向项目包的指针，用于存储插入后的项目
    /// @param packages_order 指向项目顺序的指针，用于更新项目的顺序
    /// @return 如果插入成功返回 true，否则返回 false
    static bool insertItem(const RobotState::Item& item, RobotState::Packages* packages, RobotState::PackagesOrder* packages_order);
};

#endif