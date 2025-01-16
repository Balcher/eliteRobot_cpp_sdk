#include "RobotInterface.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include "Robot40011Port.hpp"

int main()
{

    // 40011端口
	std::unique_ptr<Robot40011Port> robot = std::make_unique<Robot40011Port>();    // 创建机器人接口实例
	std::string ipAdd = "192.168.205.133";    // ip地址

	if (!robot->connect(ipAdd, 40011))
	{
		std::cout << "Connect fail check network or IP" << std::endl;
		return -1;
	}

	// 30001端口
   std::unique_ptr<RobotInterface> robot2 = std::make_unique<RobotInterface>();    // 创建机器人接口实例

   if (!robot2->loadConfigure("CS_UserManual_Robot_State_Message.txt")) {          // 加载配置文件
       std::cout << "Load Configure file fail. Check file path" << std::endl;
       return -1;
   }
   robot2->setExceptionCallback([](const RobotException& exception) {               // 异常处理回调
       // do somethings when an exception occurs.
       std::cout << "time stamp: " << exception.timestamp << " ";
       if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
           std::cout << "Runtime exception" << std::endl;
       }
       else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE) {
           std::cout << "Robot error exception" << std::endl;
       }
       });
   if (!robot2->connect(ipAdd, 30001)) {                                          // 连接到机器人
       std::cout << "Connect fail check network or IP" << std::endl;
       return -1;
   }


    // 逆解运算
	std::vector<double> poses = { 0.310, -0.07535, 2.02347, -1.470, 0.343, -1.016 };
	std::vector<double> degrees;
	bool flag = robot->getIKDegrees(poses, degrees);
	std::cout << "flag = " << flag << std::endl;
	for (auto degree : degrees)
	{
		std::cout << "degree = " << degree << std::endl;
	}

    // 执行运动
    if (flag == true)
    {
        double acceleration = 1.4;
        double velocity = 0.5;
        double time = 0;
        double r = 0;

        robot2->robotMove(degrees, acceleration, velocity, time, r);
    }


}





