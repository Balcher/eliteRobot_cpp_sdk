#include "RobotException.hpp"
#include "RobotState.hpp"
#ifdef __linux__
#include <netinet/in.h>
#elif defined(_WIN32)
#include <winsock2.h>
#endif



#pragma pack(1)
struct RobotExceptionMessageHead {
    std::uint32_t len;
    std::uint8_t type;
    std::uint64_t timestamp;
    RobotException::Source source;
    RobotException::ExceptionType exception_type;
};

struct RobotExceptionRuntime {
    RobotExceptionMessageHead head;
    std::int32_t script_line;
    std::int32_t script_column;
    char text[0];
};

struct RobotExceptionErrorCode {
    RobotExceptionMessageHead head;
    std::int32_t error_code;
    std::int32_t error_subcode;
    RobotException::ErrorLevel level;
    RobotException::DataType data_type;
    uint8_t data_bytes[0];
};

#pragma pack()

template<typename T>
static T flipBytes(T data) {
    T temp = data;
    char *pdata = (char *)&temp;
    constexpr int size = sizeof(T);
    constexpr int count = size / 2;
    for (int i = 0; i < count; i++) {
        int j = size - i - 1;
        pdata[i] = pdata[i] ^ pdata[j];
        pdata[j] = pdata[i] ^ pdata[j];
        pdata[i] = pdata[i] ^ pdata[j];
    }
    return temp;
}


RobotException::RobotException()
{

}

RobotException::~RobotException()
{
}


std::shared_ptr<RobotException> RobotException::unpackException(const uint8_t *buffer) {
    const RobotExceptionMessageHead* head = static_cast<const RobotExceptionMessageHead*>(static_cast<const void*>(buffer));
    std::shared_ptr<RobotException> result = std::make_shared<RobotException>();
    result->timestamp = flipBytes(head->timestamp);
    result->source = head->source;
    result->exception_type = head->exception_type;
    if (head->exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
        const RobotExceptionRuntime* exception = static_cast<const RobotExceptionRuntime*>(static_cast<const void*>(buffer));
        result->runtime_exception.script_line = htonl(exception->script_line);
        result->runtime_exception.script_column = htonl(exception->script_column);
        int text_len = head->len - sizeof(RobotExceptionRuntime);
        if (text_len < 0) {
            return nullptr;
        }
        std::unique_ptr<char[]> exception_text = std::make_unique<char[]>(text_len + 1);
        memcpy(exception_text.get(), &buffer[sizeof(RobotExceptionRuntime)], text_len);
        result->runtime_exception.runtime_exception_text = exception_text.get();
    } else if (head->exception_type == RobotException::ExceptionType::ERROR_CODE) {
        const RobotExceptionErrorCode* exception = static_cast<const RobotExceptionErrorCode*>(static_cast<const void*>(buffer));
        result->error_exception.data_type = (DataType)htonl(exception->data_type);
        result->error_exception.level = (ErrorLevel)htonl(exception->level);
        result->error_exception.code = htonl(exception->error_code);
        result->error_exception.subcode = htonl(exception->error_subcode);
        if(result->error_exception.data_type == DataType::DATA_TYPE_NONE) {
            uint32_t *temp = (uint32_t*)exception->data_bytes;
            result->error_exception.exception_data = htonl(*temp);
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_UNSIGNED) {
            uint32_t *temp = (uint32_t*)exception->data_bytes;
            result->error_exception.exception_data = htonl(*temp); 
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_SIGNED) {
            int32_t *temp = (int32_t*)exception->data_bytes;
            result->error_exception.exception_data = htonl(*temp);
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_FLOAT) {
            float *temp = (float*)exception->data_bytes;
            result->error_exception.exception_data = flipBytes(*temp);
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_HEX) {
            uint32_t *temp = (uint32_t*)exception->data_bytes;
            result->error_exception.exception_data = htonl(*temp); 
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_STRING) {
            int text_len = head->len - sizeof(RobotExceptionErrorCode);
            if (text_len < 0) {
                return nullptr;
            }
            std::unique_ptr<char[]> exception_text = std::make_unique<char[]>(text_len + 1);
            memcpy(exception_text.get(), &buffer[sizeof(RobotExceptionErrorCode)], text_len);
            result->error_exception.exception_data = std::string(exception_text.get());
        } else if(result->error_exception.data_type == DataType::DATA_TYPE_JOINT) {
            int32_t *temp = (int32_t*)exception->data_bytes;
            result->error_exception.exception_data = htonl(*temp);
        }
    }
    return result;
}
