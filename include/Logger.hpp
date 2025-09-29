#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <string>
#include <sstream>
#include <iostream>
#include <memory>

#define LOGGER_MAX_BUFFER_SIZE 8192

class Logger
{
public:
    static Logger& getInstance();

    enum Level {
        LOG_INFO,
        LOG_WARNING,
        LOG_ERROR
    };


    template<Logger::Level level, typename... Args>
    void log(const std::string& fmt, const char* func_name, int line, Args... args) {
        char temp_buf[LOGGER_MAX_BUFFER_SIZE] = { 0 };
        std::snprintf(temp_buf, sizeof(temp_buf), fmt.c_str(), args...);
        log<level>(temp_buf, func_name, line);
    }

    template<Logger::Level level>
    void log(const std::string& msg, const char* func_name, int line) {
        char buf[LOGGER_MAX_BUFFER_SIZE] = { 0 };
        constexpr const char* log_msg_fmt = "[%s] - [\n\n%s\n\n] - [%s - line: %d]\n\n\n";
        std::snprintf(buf, sizeof(buf), log_msg_fmt, logLevelString<level>(), msg.c_str(), func_name, line);
        std::cout << buf;
    }
    Logger();
    ~Logger();

    // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    class DisperseMsg
    {
    private:
        static constexpr size_t MAX_MSG_LEN = LOGGER_MAX_BUFFER_SIZE;
        std::unique_ptr<char[]> buf;
        size_t buf_use_count;
    public:
        template<typename... Args>
        void append(const std::string& fmt, Args... args) {
            buf_use_count += snprintf(&buf.get()[buf_use_count], (MAX_MSG_LEN - buf_use_count - 1), fmt.c_str(), args...);
        }

        void append(const std::string& fmt);

        const char* get_msg();

        DisperseMsg();
        ~DisperseMsg();

        // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
        DisperseMsg(const DisperseMsg&) = delete;
        DisperseMsg& operator=(const DisperseMsg&) = delete;
        DisperseMsg(DisperseMsg&&) = delete;
        DisperseMsg& operator=(DisperseMsg&&) = delete;
    };

private:
    template<Level level>
    constexpr const char* logLevelString() {
        switch (level) {
        case LOG_INFO: return "INFO";
        case LOG_WARNING: return "WARNING";
        case LOG_ERROR: return "ERROR";
        default: return "UNKNOW";
        }
    }
    static void initLogger();
};

template<typename ...Args>
static inline void INFO_LOG(const char* fmt, Args... args) {
    Logger::getInstance().log<Logger::LOG_INFO>(fmt, __func__, __LINE__, args...);
}

template<typename ...Args>
static inline void WARNING_LOG(const char* fmt, Args... args) {
    Logger::getInstance().log<Logger::LOG_WARNING>(fmt, __func__, __LINE__, args...);
}

template<typename ...Args>
static inline void ERROR_LOG(const char* fmt, Args... args) {
    Logger::getInstance().log<Logger::LOG_ERROR>(fmt, __func__, __LINE__, args...);
}

#endif