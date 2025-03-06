#include "Robot40011Port.hpp"

#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <regex>
#include <vector>
#include <sstream>

#ifdef __linux__
#include <netinet/in.h>
#elif defined(_WIN32)
#include <winsock2.h>
#endif

struct Robot40011Port::Socket {
	std::unique_ptr<boost::asio::io_service> io_service_ptr;
	std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;
	std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr;
};


Robot40011Port::Robot40011Port()
{
	connection_state = DISCONNECTED;
	robot_socket = std::make_unique<Socket>();
	robot_socket->io_service_ptr = std::make_unique<boost::asio::io_service>(1); // 1：单线程模式
	robot_socket->socket_ptr = std::make_unique<boost::asio::ip::tcp::socket>(*robot_socket->io_service_ptr);
	robot_socket->resolver_ptr = std::make_unique<boost::asio::ip::tcp::resolver>(*robot_socket->io_service_ptr);
}

Robot40011Port::~Robot40011Port()
{
	disconnect();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief 提取socket接收到的string类型中有用的东西
/// Received: [1] [success] : [True]
/// @param input 接收到的数据
/// @return 有效信息
std::vector<std::string> extracValues(const std::string& input)
{
	std::vector<std::string> values;
	std::regex regex(R"(\[([^\[\]]*?)\])"); // 正则表达式匹配方括号内的内容
	std::smatch match;

	// 使用 std::sregex_iterator 遍历所有匹配项
	//auto it = std::sregex_iterator(input.begin(), input.end(), regex);
	auto end = std::sregex_iterator();

	for (auto it = std::sregex_iterator(input.begin(), input.end(), regex); it != end; ++it) {
		values.push_back(it->str(1)); // 提取第一个捕获组
	}
	return values;
}

/// @brief 根据角度值生成规范的messages
/// @param command 命令
/// @param poses 位姿数据
/// @return 通信信息
std::string generateMessages(std::string command,const std::vector<double>& poses)
{
	std::ostringstream script;
	script << "req 1 ";
	script << command;
	script << "([";

	//添加角度值
	for (size_t i = 0; i < poses.size(); ++i)
	{
		script << poses[i];
		if (i < poses.size() - 1) {
			script << ",";
		}
	}
	script << "])\n";
	return script.str();
}

/// @brief 从buffer中提取角度信息
/// @param buffer 带有角度信息的字符串
/// @return 角度信息数组
std::vector<double> getdegreesMessage(std::string buffer)
{
	std::vector<double> numbers;
	std::regex regex(R"(-?\d+\.\d+|-?\d+)");  // 正则表达式匹配数字

	auto end = std::sregex_iterator();
	for (auto it = std::sregex_iterator(buffer.begin(), buffer.end(), regex); it != end; ++it)
	{
		numbers.push_back(std::stof(it->str())); // 将提取的字符串转换成 float
	}
	return numbers;
}

bool Robot40011Port::getIKDegrees(const std::vector<double>& poses, std::vector<double>& degrees)
{

	// 1. 查询是否存在逆解

	// 生成查询字符串
	std::string message = generateMessages("get_inverse_kin_has_solution", poses);

	// 发送指令
	this->sendScript(message);

	// 接收指令
	const int buffer_size = 100;
	std::string buffer(buffer_size, '\0');
	this->socketRecv(buffer.data(), buffer_size);
	std::cout << "Received : " << buffer.data() << std::endl;
	// 提取有用信息
	std::vector<std::string> values;
	values = extracValues(buffer);

	// 判断是否存在解
	if (values[2] != "True")
		return false;

    
	// 2. 查询逆解的电机角度值

	// 生成查询字符串
	message = generateMessages("get_inverse_kin", poses);
	std::cout << message << std::endl;

	// 发送指令 
	this->sendScript(message);

	// 接收指令
	this->socketRecv(buffer.data(), buffer_size);
	std::cout << "Received : " << buffer.data() << std::endl;

	// 提取有用信息
	values = extracValues(buffer);

    // 提取角度信息
	degrees = getdegreesMessage(values[2]);

	return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////

bool Robot40011Port::connect(const std::string& ip, int port)
{
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

	robot_socket->socket_ptr->set_option(boost::asio::socket_base::reuse_address(true), ec); // 设置套接字选项以允许地址重用
	if (ec) {
		robot_socket->socket_ptr->close();
		return false;
	}

	robot_socket->socket_ptr->set_option(boost::asio::socket_base::keep_alive(true), ec); // 设置套接字以启用TCP的保持活动选项
	if (ec) {
		robot_socket->socket_ptr->close();
		return false;
	}

	connection_state = CONNECTED;
	return true;
}

void Robot40011Port::disconnect()
{
	std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
	if (!robot_socket->socket_ptr)
	{
		connection_state = DISCONNECTED;
		return;
	}

	if (isConnect())
	{
		try
		{
			robot_socket->socket_ptr->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
		}
		catch (const std::exception& e)
		{
			std::cout << "Socket shutdown has exception " << e.what() << std::endl;
		}
		robot_socket->socket_ptr->close();
	}
	connection_state = DISCONNECTED;

}

int Robot40011Port::sendScript(const std::string& script)
{
	boost::asio::write(*robot_socket->socket_ptr, boost::asio::buffer(script));
	//return robot_socket->socket_ptr->send(boost::asio::buffer(script));
	return 1;
}


int Robot40011Port::socketRecv(void* out_msg, int size)
{
	int rl = 0;
	std::lock_guard<std::mutex> socket_lock(socket_instance_mutex);
	boost::system::error_code ec;    // 捕获错误信息
	// boost::asio::buffer(out_msg, size) 创建了一个缓冲区，指向out_msg指向的内存区域，并指定其大小为 size
	rl = (int)robot_socket->socket_ptr->read_some(boost::asio::buffer(out_msg, size), ec);
	if (ec)
	{
		disconnect();
		rl = -1;
	}
	return rl;
}


bool Robot40011Port::isConnect() { return (connection_state == CONNECTED); }