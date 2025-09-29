#ifndef __ROBOTEXCEPTION_HPP__
#define __ROBOTEXCEPTION_HPP__

#include <memory>
#include <any>
#include <string>
#include <cstdint> 

/// @brief RobotException 定义了一个RobotException类。
/// @note 用于处理与机器人相关的异常
class RobotException
{
private:

public:
    RobotException();
    ~RobotException();

    /// @brief 从字节缓冲区解包异常
    /// 
    /// 该方法解析给定的字节缓冲区，提取序列化的异常数据，并返回一个
    /// 'RobotException'对象的智能指针。如果解包过程中出现错误，
    /// 将返回'nullptr'
    /// 
    /// @param buffer 指向包含序列化异常数据的字节缓冲区
    /// @return 返回解包后的'RobotException'对象的智能指针；如果解包失败，则返回'nullptr'
    static std::shared_ptr<RobotException> unpackException(const uint8_t* buffer);

    std::uint64_t timestamp; ///< 记录异常发生的时间戳

    /// @brief 异常类型枚举
    enum ExceptionType : std::uint8_t {
        RUN_TIME_EXCEPTION = 10,        ///< 运行时异常
        ERROR_CODE = 6                  ///< 错误代码
    };
    ExceptionType exception_type;

    /// @brief 异常来源枚举，包括运行时、安全、GUI、控制器等
    enum Source : std::uint8_t {
        SOURCE_RUNTIME = 10,
        SOURCE_SAFETY = 99,             ///< 安全控制器
        SOURCE_GUI = 103,               ///< 示教器UI
        SOURCE_CONTROLLER = 104,        ///< 控制器
        SOURCE_RTSI = 105,              ///< RTSI协议
        SOURCE_JOINT = 120,             ///< 关节
        SOURCE_TOOL = 121,              ///< 工具
        SOURCE_TP = 122,                ///< 示教器
        SOURCE_JOINT_FPGA = 200,        ///< 关节FPGA
        SOURCE_TOOL_FPGA = 201          ///< 工具FPGA
    };
    Source source;

    /// @brief 运行时异常结构
    struct {
        int32_t script_line;                 ///< 脚本行号
        int32_t script_column;               ///< 脚本列号
        std::string runtime_exception_text;  ///< 运行时异常文本
    } runtime_exception;

    enum ErrorLevel : std::uint32_t{
        INFO = 0,
        WARNING = 1,
        ERROR = 2,
        SEGMENT_FAULT = 3
    };

    /// @brief 定义了不同的数据类型
    enum DataType : std::uint32_t {
        DATA_TYPE_NONE = 0,            ///< 无
        DATA_TYPE_UNSIGNED = 1,        ///< 无符号
        DATA_TYPE_SIGNED = 2,          ///< 带符号
        DATA_TYPE_FLOAT = 3,           ///< 浮点
        DATA_TYPE_HEX = 4,             ///< 十六进制
        DATA_TYPE_STRING = 5,          ///< 字符串
        DATA_TYPE_JOINT = 6            ///< 关节数据
    };

    /// @brief 异常的详细信息
    struct {
        int32_t code;                  ///< 错误代码
        int32_t subcode;               ///< 子代码
        ErrorLevel level;              ///< 错误级别
        DataType data_type;            ///< 数据类型
        std::any exception_data;       ///< 任意异常数据
    } error_exception;

};

#endif
