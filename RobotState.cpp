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

void RobotState::recv_thread() {
    std::unique_ptr<uint8_t[]> recv_buff = std::make_unique<uint8_t[]>(40960);
    while (recv_thread_keep_alive) {
        updateRobotState(recv_buff.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotState::setExceptionCallback(const std::function<void(const RobotException&)>& callback) {
    robot_exception_callback = callback;
}

bool RobotState::connect(const std::string& ip, int port) {
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
    boost::system::error_code ec;
    robot_socket->socket_ptr->open(boost::asio::ip::tcp::v4(), ec);
    if (ec) {
        return false;
    }
    boost::asio::ip::tcp::resolver::query query(ip, std::to_string(port));
    boost::asio::connect(*robot_socket->socket_ptr, robot_socket->resolver_ptr->resolve(query), ec);
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->set_option(boost::asio::ip::tcp::no_delay(true), ec);
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->non_blocking(false, ec);
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }
    robot_socket->socket_ptr->set_option(boost::asio::socket_base::keep_alive(true), ec);
    if (ec) {
        robot_socket->socket_ptr->close();
        return false;
    }

    recv_thread_keep_alive = true;
    recv_thread_handle = std::make_unique<std::thread>([&]() { recv_thread(); });
    if (!recv_thread_handle) {
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

void RobotState::updateRobotState(uint8_t* recv_buff) {
    if (!isConnect()) {
        return;
    }
    MessageHead last_head;
    while (true) {
        MessageHead head = {0};
        if (socketRecv(&head, sizeof(head)) != sizeof(head)) {
            return;
        }
        if (!isConnect()) {
            return;
        }
        head.len = htonl(head.len);
        if (head.len <= 0) {
            std::cout << "Message error last pack: len->" << last_head.len << " type->" << last_head.type << std::endl;
            disconnect();
            return;
        }
        last_head = head;
        if (head.type == ROBOT_STATE_MESSAGE_TYPE) {
            memcpy(recv_buff, &head, sizeof(head));
            socketRecv(&recv_buff[sizeof(head)], head.len - sizeof(head));
            WriteLock(packages_mutex);
            convertByteStream(recv_buff, head.len);
            break;
        } else if(head.type == ROBOT_EXCEPTION_MESSAGE_TYPE) {
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

int RobotState::socketRecv(void* out_msg, int size) {
    int rl = 0;
    std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
    boost::system::error_code ec;
    rl = (int)robot_socket->socket_ptr->receive(boost::asio::buffer(out_msg, size), MSG_WAITALL, ec);
    if (ec) {
        disconnect();
        rl = -1;
    }
    return rl;
}

bool RobotState::isConnect() { return (connection_state == CONNECTED); }