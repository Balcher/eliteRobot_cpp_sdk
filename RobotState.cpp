#include "RobotState.hpp"
#include "RobotException.hpp"

#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#ifdef __linux__
#include <netinet/in.h>
#elif defined(_WIN32)
#include <winsock2.h>
#endif

#pragma pack(1)
struct MessageHead {
    std::uint32_t len;
    std::uint8_t type;
};
#pragma pack()

struct RobotState::Sokcet {
    std::unique_ptr<boost::asio::io_service> io_service_ptr;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr;
};

#define ROBOT_STATE_MESSAGE_TYPE (16)
#define ROBOT_EXCEPTION_MESSAGE_TYPE (20)

RobotState::RobotState() {
    recv_thread_keep_alive = false;
    connection_state = DISCONNECTED;
    robot_socket = std::make_unique<Sokcet>();
    robot_socket->io_service_ptr = std::make_unique<boost::asio::io_service>(1);  // 1: 单线程模式
    robot_socket->socket_ptr = std::make_unique<boost::asio::ip::tcp::socket>(*robot_socket->io_service_ptr);
    robot_socket->resolver_ptr = std::make_unique<boost::asio::ip::tcp::resolver>(*robot_socket->io_service_ptr);
}

RobotState::~RobotState() {
    recv_thread_keep_alive = false;
    disconnect();
}

/// @brief 一个接收数据的线程，不断更新机器人的状态
/// 主要功能：持续接收数据并更新机器人的状态
void RobotState::recv_thread() { // 定义了一个返回类型为 void 的成员函数 recv_thread，没有参数
    // 设置接收缓冲区，创建一个动态分配的数组 recv_buff，大小为40960（约40KB），用来存储接收到的数据
    // 这种智能指针确保在超出作用域时自动释放内存。
    std::unique_ptr<uint8_t[]> recv_buff = std::make_unique<uint8_t[]>(40960);
    // 使用while循环，当 recv_thread_keep_alive为true时，持续执行循环体，这使得线程可以在需要时安全地停止
    while (recv_thread_keep_alive) {
        // 调用updateRobotState方法，将接收缓冲区的指针传递给它。这个方法应该负责处理接收到的数据并更新机器人的状态
        updateRobotState(recv_buff.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotState::setExceptionCallback(const std::function<void(const RobotException&)>& callback) {
    robot_exception_callback = callback;
}

/// @brief 建立与指定IP地址和端口的TCP连接
/// @param ip IP地址
/// @param port 端口号
/// @return true：连接成功，false：连接失败
bool RobotState::connect(const std::string& ip, int port) {
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex); // 锁定互斥量
    boost::system::error_code ec; // 错误代码，用于捕获在连接过程中可能发生的错误
    robot_socket->socket_ptr->open(boost::asio::ip::tcp::v4(), ec); // 尝试打开一个IPv4的TCP套接字，如果发生错误，将其记录在ec中
    if (ec) {
        return false; // 连接失败
    }
    boost::asio::ip::tcp::resolver::query query(ip, std::to_string(port)); // 创建一个解析查询对象 query，用于解析给定的IP地址和端口号
    boost::asio::connect(*robot_socket->socket_ptr, robot_socket->resolver_ptr->resolve(query), ec); // 使用解析查询的结果尝试连接到服务器，如果发生错误，记录在ec中
    if (ec) {
        robot_socket->socket_ptr->close(); // 如果连接失败，关闭套接字并返回 false
        return false;
    }
    // set_option：设置套接字选项
    robot_socket->socket_ptr->set_option(boost::asio::socket_base::reuse_address(true), ec); // 设置套接字选项以允许地址重用
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->set_option(boost::asio::ip::tcp::no_delay(true), ec); // 启用TCP的Nagle算法禁用，以减少延迟
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->non_blocking(false, ec); // 将套接字设置为阻塞模式
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->set_option(boost::asio::socket_base::keep_alive(true), ec); // 设置套接字以启用TCP的保持活动选项
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }

    recv_thread_keep_alive = true; // 设置 recv_thread_keep_alive 为 true，并启动一个新的线程来处理接收数据，调用 recv_thread 方法
    recv_thread_handle = std::make_unique<std::thread>([&]() { recv_thread(); }); // 启动一个新的线程来处理接收数据，调用recv_thread方法
    if (!recv_thread_handle) { // 如果线程创建失败，返回 false
        return false;
    }
    connection_state = CONNECTED;
    return true;
}

void RobotState::disconnect() {
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
    if (!robot_socket->socket_ptr) {
        connection_state = DISCONNECTED;
        return;
    }
    if (isConnect()) {
        try {
            robot_socket->socket_ptr->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        } catch (const std::exception& e) {
            std::cout << "Socket shutdown has exception " << e.what() << std::endl;
        }
        robot_socket->socket_ptr->close();
    }
    connection_state = DISCONNECTED;
    recv_thread_keep_alive = false;
    if (recv_thread_handle && recv_thread_handle->joinable()) {
        recv_thread_handle->join();
    }
}

int RobotState::sendScript(const std::string& script) {
    return robot_socket->socket_ptr->send(boost::asio::buffer(script.c_str(), script.size()), 0);
}

bool RobotState::abortMessageRecv(int size) {
    std::unique_ptr<uint8_t[]> temp_ptr = std::make_unique<uint8_t[]>(size);
    socketRecv(temp_ptr.get(), size);
    return true;
}

void RobotState::flipBytes(void* data, int size) {
    char* pdata = (char*)data;
    for (int i = 0; i < (size / 2); i++) {
        int j = size - i - 1;
        pdata[i] = pdata[i] ^ pdata[j];
        pdata[j] = pdata[i] ^ pdata[j];
        pdata[i] = pdata[i] ^ pdata[j];
    }
}

bool RobotState::convertByteStream(const uint8_t* bytes, uint32_t bytes_len) {
    Packages::iterator iter;
    uint32_t offset = 0;
    for (auto& item_key : packages_order) {
        iter = packages.find(item_key);
        if (iter == packages.end()) {
            return false;
        }
        Item& item = packages[item_key];
        memcpy(item.data_buff, &bytes[offset], item.size);
        flipBytes(item.data_buff, item.size);
        offset += item.size;
        if (offset > bytes_len) {
            return false;
        }
    }
    return true;
}

/// @brief 用于更新机器人的状态，从套接字接收数据，并根据接收到的消息类型进行相应处理
/// @param recv_buff 指向uint8_t类型的缓冲区指针 recv_buff
void RobotState::updateRobotState(uint8_t* recv_buff) {
    // 如果当前未连接，直接返回，停止执行后续操作
    if (!isConnect()) {
        return;
    }

    MessageHead last_head; // 用于保存上一个接收的消息头
    while (true) {
        MessageHead head = {0}; // 初始化消息头：创建并初始化 一个新的head变量
        // 接收消息头：从套接字接收消息头，如果接收的字节数不等于 messageHead的大小，返回
        if (socketRecv(&head, sizeof(head)) != sizeof(head)) {
            return;
        }
        if (!isConnect()) { // 检查连接状态，确保在接收过程中仍然保持连接
            return;
        }
        head.len = htonl(head.len); // 将接收到的消息头中长度字段 len 从网络字节序转换为主机字节序
        // 如果长度不合法（小于或等于0），输出错误信息，断开连接并返回
        if (head.len <= 0) {
            std::cout << "Message error last pack: len->" << last_head.len << " type->" << last_head.type << std::endl;
            disconnect();
            return;
        }
        last_head = head; // 更新上一个消息头，将当前的消息头赋值给 last_head，以便后续使用
        // 检查消息的类型是否为机器人状态消息
        if (head.type == ROBOT_STATE_MESSAGE_TYPE) {
            memcpy(recv_buff, &head, sizeof(head));                         // 将当前消息头的内容复制到接收缓冲区 recv_buff 中
            socketRecv(&recv_buff[sizeof(head)], head.len - sizeof(head));  // 根据消息头中的长度信息，从套接字接收消息体并存储到缓冲区中
            // 在锁定 packages_mutex的情况下，调用convertByteStream 方法处理接收到的数据，然后跳出循环，结束消息接收
            WriteLock(packages_mutex);                                     
            convertByteStream(recv_buff, head.len);                         
            break;
        } 
        // 检查消息的类型是否为机器人异常消息
        else if(head.type == ROBOT_EXCEPTION_MESSAGE_TYPE) {
            memcpy(recv_buff, &head, sizeof(head));
            socketRecv(&recv_buff[sizeof(head)], head.len - sizeof(head));
            std::shared_ptr<RobotException> exception = RobotException::unpackException(recv_buff);
            if (robot_exception_callback) {
                robot_exception_callback(*exception);
            }
            
        } else {
            abortMessageRecv(head.len - sizeof(head));
            continue;
        }
        
    }
}

bool RobotState::loadConfigure(const std::string& file_path) { return LoadFile::loadFile(file_path, &packages, &packages_order); }

bool RobotState::findItem(const std::string& name, Item* item) {
    ReadLock(packages_mutex);
    Packages::iterator iter = packages.find(name);
    if (iter != packages.end() && item != nullptr) {
        *item = packages[name];
        return true;
    }
    return false;
}

/// @brief 从机器人socket接收数据
/// @param out_msg 用于存储接收到消息的输出缓冲区
/// @param size 要接收的数据大小（字节数）
/// @return 表示接收到的字节数，发生错误则返回 -1
int RobotState::socketRecv(void* out_msg, int size) {
    int rl = 0;
    // 创建一个锁 socket_lock ，确保在接收数据时对套接字的访问是线程安全的。
    // 这意味着在此函数执行期间，其他线程无法访问同一套套接字
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
    boost::system::error_code ec; // 定义一个 boost::system::error_code 类型的变量 ec, 用于捕获错误信息
    rl = (int)robot_socket->socket_ptr->receive(boost::asio::buffer(out_msg, size), MSG_WAITALL, ec);
    if (ec) {
        disconnect();
        rl = -1;
    }
    return rl;
}

/// @brief 从机器人socket接收数据
/// @param out_msg 用于存储接收到消息的输出缓冲区
/// @param size 要接收的数据大小（字节数）
/// @return 表示接收到的字节数，发生错误则返回 -1
int RobotState::mysocketRecv(void* out_msg, int size) {
    int rl = 0;
    // 创建一个锁 socket_lock ，确保在接收数据时对套接字的访问是线程安全的。
    // 这意味着在此函数执行期间，其他线程无法访问同一套套接字
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
    boost::system::error_code ec; // 定义一个 boost::system::error_code 类型的变量 ec, 用于捕获错误信息
    rl = (int)robot_socket->socket_ptr->read_some(boost::asio::buffer(out_msg, size), ec);
    if (ec) {
        disconnect();
        rl = -1;
    }
    return rl;
}

bool RobotState::isConnect() { return (connection_state == CONNECTED); }