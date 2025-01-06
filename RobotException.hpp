#ifndef __ROBOTEXCEPTION_HPP__
#define __ROBOTEXCEPTION_HPP__

#include <memory>
#include <any>
#include <string>

/// @brief RobotException ������һ��RobotException�ࡣ
/// @note ���ڴ������������ص��쳣
class RobotException
{
private:
    
public:
    RobotException();
    ~RobotException();

    /// @brief ���ֽڻ���������쳣
    /// 
    /// �÷��������������ֽڻ���������ȡ���л����쳣���ݣ�������һ��
    /// 'RobotException'���������ָ�롣�����������г��ִ���
    /// ������'nullptr'
    /// 
    /// @param buffer ָ��������л��쳣���ݵ��ֽڻ�����
    /// @return ���ؽ�����'RobotException'���������ָ�룻������ʧ�ܣ��򷵻�'nullptr'
    static std::shared_ptr<RobotException> unpackException(const uint8_t *buffer);
    
    std::uint64_t timestamp; ///< ��¼�쳣������ʱ���

    /// @brief �쳣����ö��
    enum ExceptionType : std::uint8_t {
        RUN_TIME_EXCEPTION = 10,        ///< ����ʱ�쳣
        ERROR_CODE = 6                  ///< �������
    };
    ExceptionType exception_type;

    /// @brief �쳣��Դö�٣���������ʱ����ȫ��GUI����������
    enum Source : std::uint8_t {
        SOURCE_RUNTIME = 10,
        SOURCE_SAFETY = 99,             ///< ��ȫ������
        SOURCE_GUI = 103,               ///< ʾ����UI
        SOURCE_CONTROLLER = 104,        ///< ������
        SOURCE_RTSI = 105,              ///< RTSIЭ��
        SOURCE_JOINT = 120,             ///< �ؽ�
        SOURCE_TOOL = 121,              ///< ����
        SOURCE_TP = 122,                ///< ʾ����
        SOURCE_JOINT_FPGA = 200,        ///< �ؽ�FPGA
        SOURCE_TOOL_FPGA = 201          ///< ����FPGA
    };
    Source source;

    /// @brief ����ʱ�쳣�ṹ
    struct {
        int32_t script_line;                 ///< �ű��к�
        int32_t script_column;               ///< �ű��к�
        std::string runtime_exception_text;  ///< ����ʱ�쳣�ı�
    } runtime_exception;
    
    /// @brief ��ͬ�Ĵ��󼶱�
    enum ErrorLevel : std::int32_t {
        INFO = 0,                      ///< ��Ϣ
        WARNING = 1,                   ///< ����
        ERROR = 2,                     ///< ����
        SEGMENT_FAULT = 3              ///< ���ش���
    };
    
    /// @brief �����˲�ͬ����������
    enum DataType : std::uint32_t {
        DATA_TYPE_NONE = 0,            ///< ��
        DATA_TYPE_UNSIGNED = 1,        ///< �޷���
        DATA_TYPE_SIGNED = 2,          ///< ������
        DATA_TYPE_FLOAT = 3,           ///< ����
        DATA_TYPE_HEX = 4,             ///< ʮ������
        DATA_TYPE_STRING = 5,          ///< �ַ���
        DATA_TYPE_JOINT = 6            ///< �ؽ�����
    };
    
    /// @brief �쳣����ϸ��Ϣ
    struct {
        int32_t code;                  ///< �������
        int32_t subcode;               ///< �Ӵ���
        ErrorLevel level;              ///< ���󼶱�
        DataType data_type;            ///< ��������
        std::any exception_data;       ///< �����쳣����
    } error_exception;

};

#endif
