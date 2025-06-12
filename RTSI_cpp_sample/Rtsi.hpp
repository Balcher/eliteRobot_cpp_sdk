#ifndef __RTSI_HPP__
#define __RTSI_HPP__

#include <unordered_map>
#include <list>
#include <memory>
#include <array>
#include "boost/asio.hpp"
#include "RtsiMsg.hpp"

class Rtsi
{
public:

    enum Command {
        PROTOCOL_VERSION_VERIFY = 'V',
        GET_ELISERVER_VERSION = 'v',
        TEXT_MESSAGE = 'M',
        DATA_PACKAGE = 'U',
        SUBSCRIBE_OUTPUTS = 'O',
        SUBSCRIBE_INPUTS = 'I',
        CONTROL_PACKAGE_START = 'S',
        CONTROL_PACKAGE_PAUSE = 'P',
    };

    typedef boost::system::error_code ErrorCode;

    bool connect(const std::string& ip, int port, ErrorCode* ec = nullptr);

    bool connect(const std::string& ip);

    void disconnect();

    bool versionCheck();

    struct ControllerVsersion {
        uint32_t major;
        uint32_t minor;
        uint32_t bugfix;
        uint32_t build;
    };
    ControllerVsersion controllerVersionGet(bool need_conver_to_little_endian = true);

    bool start();

    bool pause();

    enum ConnectionState {
        DISCONNECTED,
        CONNECTED,
        STARTED,
        STOPED
    };

    ConnectionState state();

    struct DataObject;
    class DataRecipe;
    typedef std::shared_ptr<DataRecipe> DataRecipePtr;

    DataRecipePtr outputSubscribe(const std::string& items, double frequency, ErrorCode* ec = nullptr);

    DataRecipePtr inputSubscribe(const std::string& items, ErrorCode* ec = nullptr);

    bool getOutputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec = nullptr);

    DataRecipePtr& getOutputDataToRecipe(ErrorCode* ec = nullptr);

    bool getOutPutDataToRaw(RtsiMsg::DataTxRx* data_buff, int max_buff_size, ErrorCode* ec = nullptr);

    bool sendInputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec = nullptr);

    bool sendInputDataFromRaw(const RtsiMsg::DataTxRx& data_buff, int max_buff_size, ErrorCode* ec = nullptr);

    enum class MessageType : uint8_t {
        EXCEPTION_MESSAGE = 0,
        ERROR_MESSAGE = 1,
        WARNING_MESSAGE = 2,
        INFO_MESSAGE = 3
    };
    bool sendTextMessage(const std::string& message, const std::string& source, MessageType type);

    Rtsi();
    ~Rtsi();

    // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
    Rtsi(const Rtsi&) = delete;
    Rtsi& operator=(const Rtsi&) = delete;
    Rtsi(Rtsi&&) = delete;
    Rtsi& operator=(Rtsi&&) = delete;
private:
    std::unique_ptr<boost::asio::io_service> io_service_ptr;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr;

    std::array<DataRecipePtr, 255> output_recipes_list;

    ConnectionState connection_state;

    static constexpr uint16_t protocol_version = 1;

    size_t socketRecv(void* out_msg, int size, ErrorCode* ec);

    bool messageRecv(Command cmd, void* buff, int max_buff_size, ErrorCode* ec);
    bool abortMessageRecv(int size);
    bool messageSend(const void* msg, int max_msg_size, ErrorCode* ec);

    static constexpr uint16_t internalHtons(uint16_t value);
    static void flipBytes(void* data, int size);

    DataRecipePtr buildRecipeFromRaw(uint8_t recipe_id, const std::string& items, const char* type);

    bool sendInputDataFromRawInternal(const RtsiMsg::DataTxRx& data_buff, int max_buff_size, ErrorCode* ec);
};

struct Rtsi::DataObject {
    union Data {
        double v6d[6];
        double v3d[3];
        int32_t v6i32[6];
        double d_64;
        uint16_t u_16;
        uint32_t u_32;
        uint64_t u_64;
        bool b_8;
        uint8_t u_8;
        int32_t i_32;
    };
    Data value;
    std::string name;
    std::string type;
};

class Rtsi::DataRecipe {
public:
    using DataKey = std::string ;
    using DataMap = std::unordered_map<DataKey, DataObject>;
    using ItemList = std::list<DataKey>;

    DataRecipe();

    ~DataRecipe();

    DataObject& operator[](const DataKey& name);

    bool is_empty();

    const ItemList& getItemList();

    uint8_t getID();

    bool build(uint8_t recipe_id, DataMap dmap, ItemList ilist);

    bool convertToMessage(RtsiMsg::DataTxRx* msg_buff, int max_buff_size);

    bool setValueFromMessage(const RtsiMsg::DataTxRx& msg_buff);

    bool has_not_found();

    bool has_in_use();

    bool find(const DataKey& name);

    // 禁用拷贝构造、拷贝赋值运算符、移动构造、移动赋值运算符
    DataRecipe(const DataRecipe&) = delete;
    DataRecipe& operator=(const DataRecipe&) = delete;
    DataRecipe(DataRecipe&&) = delete;
    DataRecipe& operator=(DataRecipe&&) = delete;

private:
    uint8_t id;
    DataMap valuemap;
    ItemList itemlist;
};


#endif