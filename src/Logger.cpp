#include "Logger.hpp"

#include <cstring>

static Logger* s_logger_prt = nullptr;

Logger::Logger()
{
}

Logger::~Logger()
{
}

void Logger::initLogger() {
    s_logger_prt = new Logger;
}

Logger& Logger::getInstance() {
    if (!s_logger_prt) {
        initLogger();
    }
    
    return *s_logger_prt;
}


Logger::DisperseMsg::DisperseMsg() {
    buf = std::make_unique<char[]>(MAX_MSG_LEN);
    buf_use_count = 0;
}


Logger::DisperseMsg::~DisperseMsg() {

}

void Logger::DisperseMsg::append(const std::string& fmt) {
    buf_use_count += snprintf(&buf.get()[buf_use_count], (MAX_MSG_LEN - buf_use_count - 1), "%s", fmt.c_str());
}

const char* Logger::DisperseMsg::get_msg() {
    return buf.get();
}