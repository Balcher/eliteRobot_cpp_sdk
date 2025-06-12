#include "Rtsi.hpp"
#include "RtsiMsg.hpp"
#include "Logger.hpp"

#ifdef __linux__
#include <netinet/in.h>
#elif defined(_WIN32)
#include <winsock2.h>
#endif

using DataRecipe = Rtsi::DataRecipe;
using DataObject = Rtsi::DataObject;
using DataRecipePtr = Rtsi::DataRecipePtr;

Rtsi::Rtsi()
{
    io_service_ptr = std::make_unique<boost::asio::io_service>(1); // 1: 单线程模式
    socket_ptr = std::make_unique<boost::asio::ip::tcp::socket>(*io_service_ptr);
    resolver_ptr = std::make_unique<boost::asio::ip::tcp::resolver>(*io_service_ptr);
}

Rtsi::~Rtsi()
{
}

bool Rtsi::connect(const std::string& ip, int port, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    socket_ptr->open(boost::asio::ip::tcp::v4(), *ec);
    if (*ec) {
        INFO_LOG(ec->message().c_str());
        return false;
    }
    boost::asio::ip::tcp::resolver::query query(ip, std::to_string(port));
    boost::asio::connect(*socket_ptr, resolver_ptr->resolve(query), *ec);
    if (*ec) {
        socket_ptr->close();
        return false;
    }
    socket_ptr->set_option(boost::asio::socket_base::reuse_address(true), *ec);
    if (*ec) {
        socket_ptr->close();
        return false;
    }
    socket_ptr->set_option(boost::asio::ip::tcp::no_delay(true), *ec);
    if (*ec) {
        socket_ptr->close();
        return false;
    }
    socket_ptr->non_blocking(false, *ec);
    if (*ec) {
        socket_ptr->close();
        return false;
    }
    socket_ptr->set_option(boost::asio::socket_base::keep_alive(true), *ec);
    if (*ec) {
        socket_ptr->close();
        return false;
    }
    connection_state = CONNECTED;
    return true;
}


bool Rtsi::connect(const std::string& ip) {
    boost::system::error_code ec;
    return connect(ip, 30004, &ec);
}


void Rtsi::disconnect() {

    if (!socket_ptr) {
        connection_state = ConnectionState::DISCONNECTED;
        return;
    }
    if (connection_state != ConnectionState::DISCONNECTED) {
        try {
            socket_ptr->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        }
        catch (const std::exception& e) {
            ERROR_LOG("RTSI shutdown has exception %s", e.what());
        }
        socket_ptr->close();
    }
    connection_state = ConnectionState::DISCONNECTED;
}

constexpr uint16_t Rtsi::internalHtons(uint16_t value) {
    return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

bool Rtsi::versionCheck() {
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Version check fail");
        return false;
    }
    constexpr RtsiMsg::ProtocolTx protocol_tx = {
        {
            Rtsi::internalHtons(sizeof(RtsiMsg::ProtocolTx)),
            Command::PROTOCOL_VERSION_VERIFY
        },
        Rtsi::internalHtons(protocol_version)
    };
    boost::system::error_code ec;
    messageSend(&protocol_tx, sizeof(protocol_tx), &ec);
    if (ec) {
        disconnect();
        return false;
    }
    RtsiMsg::ProtocolRx protocol_rx;
    messageRecv(Command::PROTOCOL_VERSION_VERIFY, &protocol_rx, sizeof(protocol_rx), &ec);
    if (ec) {
        disconnect();
        return false;
    }
    return protocol_rx.is_ok;
}

Rtsi::ControllerVsersion Rtsi::controllerVersionGet(bool need_conver_to_little_endian) {
    ControllerVsersion version = {0, 0, 0 ,0};
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Controller version get fail");
        return version;
    }
    constexpr RtsiMsg::ControllerVersionTx version_tx = {
        {
            Rtsi::internalHtons(sizeof(RtsiMsg::ControllerVersionTx)),
            Command::GET_ELISERVER_VERSION
        }
    };
    boost::system::error_code ec;
    if (!messageSend(&version_tx, sizeof(version_tx), &ec)) {
        return version;
    }
    RtsiMsg::ControllerVersionRx rx;
    if (!messageRecv(Command::GET_ELISERVER_VERSION, &rx, sizeof(rx), &ec)) {
        return version;
    }
    if (need_conver_to_little_endian) {
        version.major = htonl(rx.major);
        version.minor = htonl(rx.minor);
        version.build = htonl(rx.build);
        version.bugfix = htonl(rx.bugfix);
    }
    else {
        version.major = rx.major;
        version.minor = rx.minor;
        version.build = rx.build;
        version.bugfix = rx.bugfix;
    }
    return version;
}

bool Rtsi::start() {
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Start fail");
        return false;
    }
    constexpr RtsiMsg::StartTx start_tx = {
        {
            Rtsi::internalHtons(sizeof(RtsiMsg::StartTx)),
            Command::CONTROL_PACKAGE_START
        }
    };
    ErrorCode ec;
    if (!messageSend(&start_tx, sizeof(start_tx), &ec)) {
        return false;
    }
    RtsiMsg::StartRx start_rx;
    if (!messageRecv(Command::CONTROL_PACKAGE_START, &start_rx, sizeof(start_rx), &ec)) {
        return false;
    }
    connection_state = start_rx.is_start ? ConnectionState::STARTED : connection_state;
    return start_rx.is_start;
}

bool Rtsi::pause() {
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Pause fail");
        return false;
    }
    constexpr RtsiMsg::PauseTx pause_tx = {
        {
            Rtsi::internalHtons(sizeof(RtsiMsg::PauseTx)),
            Command::CONTROL_PACKAGE_PAUSE
        }
    };
    ErrorCode ec;
    if (!messageSend(&pause_tx, sizeof(pause_tx), &ec)) {
        return false;
    }
    RtsiMsg::PauseRx pause_rx;
    if (!messageRecv(Command::CONTROL_PACKAGE_PAUSE, &pause_rx, sizeof(pause_rx), &ec)) {
        return false;
    }
    connection_state = pause_rx.is_pause ? ConnectionState::STOPED : connection_state;
    return pause_rx.is_pause;
}

Rtsi::ConnectionState Rtsi::state() {
#ifdef __linux__
    if (connection_state != DISCONNECTED) {
        struct tcp_info tcpinfo;
        int size = sizeof(tcpinfo);
        if (getsockopt(socket_ptr->native_handle(), IPPROTO_TCP, TCP_INFO, &tcpinfo, (socklen_t*)&size)) {
            INFO_LOG("RTSI get connect state fail");
            disconnect();
        }
        if (tcpinfo.tcpi_state != TCP_ESTABLISHED) {
            INFO_LOG("RTSI tcp is not established will disconnect");
            disconnect();
        }
    }
#endif
    return connection_state;
}

DataRecipePtr Rtsi::outputSubscribe(const std::string& items, double frequency, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Subscribe output items fail.");
        return DataRecipePtr();
    }
    if (!ec) {
        return DataRecipePtr();
    }

    char buff[20480] = { 0 };
    RtsiMsg::SubscribeOutTx* sub_tx = (RtsiMsg::SubscribeOutTx*)buff;
    size_t msg_len = sizeof(RtsiMsg::SubscribeOutTx) + items.size();
    sub_tx->head.size = htons((uint16_t)msg_len);
    sub_tx->head.type = Command::SUBSCRIBE_OUTPUTS;
    sub_tx->frequency = frequency;
    memcpy(&buff[sizeof(RtsiMsg::SubscribeOutTx)], items.c_str(), items.size());
    INFO_LOG("RTSI subscribe frequency %lf", sub_tx->frequency);
    flipBytes(&sub_tx->frequency, sizeof(double));

    if (!messageSend(sub_tx, (int)msg_len, ec)) {
        return DataRecipePtr();
    }

    RtsiMsg::SubscribeRx* sub_rx = (RtsiMsg::SubscribeRx*)buff;
    memset(sub_rx, 0, sizeof(buff));
    if (!messageRecv(Command::SUBSCRIBE_OUTPUTS, sub_rx, sizeof(buff), ec)) {
        return DataRecipePtr();
    }
    output_recipes_list[sub_rx->recipe_id] = buildRecipeFromRaw(sub_rx->recipe_id, items, &buff[sizeof(RtsiMsg::SubscribeRx)]);

    return output_recipes_list[sub_rx->recipe_id];
}

DataRecipePtr Rtsi::inputSubscribe(const std::string& items, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    if (connection_state == ConnectionState::DISCONNECTED) {
        ERROR_LOG("RTSI don't connect. Subscribe output items fail.");
        return DataRecipePtr();
    }
    if (!ec) {
        return DataRecipePtr();
    }

    char buff[20480] = { 0 };

    RtsiMsg::SubscribeInTx* sub_tx = (RtsiMsg::SubscribeInTx*)buff;
    size_t msg_len = sizeof(RtsiMsg::SubscribeInTx) + items.length();
    sub_tx->head.size = htons((uint16_t)msg_len);
    sub_tx->head.type = Command::SUBSCRIBE_INPUTS;
    memcpy(&buff[sizeof(RtsiMsg::SubscribeInTx)], items.c_str(), items.size());

    if (!messageSend(sub_tx, sizeof(buff), ec)) {
        return DataRecipePtr();
    }

    RtsiMsg::SubscribeRx* sub_rx = (RtsiMsg::SubscribeRx*)buff;
    memset(sub_rx, 0, sizeof(buff));
    if (!messageRecv(Command::SUBSCRIBE_INPUTS, sub_rx, sizeof(buff), ec)) {
        return DataRecipePtr();
    }
    return buildRecipeFromRaw(sub_rx->recipe_id, items, &buff[sizeof(RtsiMsg::SubscribeRx)]);
}

bool Rtsi::getOutPutDataToRaw(RtsiMsg::DataTxRx* data_buff, int max_buff_size, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    if (connection_state != ConnectionState::STARTED) {
        ERROR_LOG("RTSI not started, can't get data");
        return false;
    }
    if (!messageRecv(Command::DATA_PACKAGE, data_buff, max_buff_size, ec)) {
        return false;
    }
    return true;
}

DataRecipePtr& Rtsi::getOutputDataToRecipe(ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    uint8_t buff[20480] = { 0 };
    if (!getOutPutDataToRaw((RtsiMsg::DataTxRx*)buff, sizeof(buff), ec)) {
        return output_recipes_list[0];
    }
    int recipe_id = ((RtsiMsg::DataTxRx*)buff)->id;
    if (!output_recipes_list[recipe_id]) {
        return output_recipes_list[0];
    }
    output_recipes_list[recipe_id]->setValueFromMessage(*(RtsiMsg::DataTxRx*)buff);
    return output_recipes_list[recipe_id];
}

bool Rtsi::getOutputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    if (!recipe_ptr) {
        return false;
    }
    uint8_t buff[20480] = { 0 };
    if (!getOutPutDataToRaw((RtsiMsg::DataTxRx*)buff, sizeof(buff), ec)) {
        return false;
    }
    return recipe_ptr->setValueFromMessage(*(RtsiMsg::DataTxRx*)buff);
}

bool Rtsi::sendInputData(const DataRecipePtr& recipe_ptr, ErrorCode* ec) {
    if (!recipe_ptr) {
        return false;
    }
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    uint8_t buff[20480] = { 0 };
    if (recipe_ptr->convertToMessage((RtsiMsg::DataTxRx*)buff, sizeof(buff))) {
        return sendInputDataFromRawInternal(*(RtsiMsg::DataTxRx*)buff, sizeof(buff), ec);
    }
    return false;
}

bool Rtsi::sendInputDataFromRaw(const RtsiMsg::DataTxRx& data_buff, int max_buff_size, ErrorCode* ec) {
    ErrorCode temp_ec;
    if (!ec) {
        ec = &temp_ec;
    }
    return sendInputDataFromRawInternal(data_buff, max_buff_size, ec);
}

bool Rtsi::sendInputDataFromRawInternal(const RtsiMsg::DataTxRx& data_buff, int max_buff_size, ErrorCode* ec) {
    return messageSend(&data_buff, max_buff_size, ec);
}

DataRecipePtr Rtsi::buildRecipeFromRaw(uint8_t recipe_id, const std::string& items, const char* type) {
    size_t pos = 0;
    DataRecipe::ItemList item_list;
    DataRecipe::DataMap item_map;
    DataObject data_obj;
    for (size_t i = 0; i < items.size();) {
        pos = items.find(',', i);
        if (pos >= std::string::npos) {
            std::string sub = items.substr(i, pos);
            item_list.push_back(sub);
            item_map[sub] = data_obj;
            break;
        }
        std::string sub = items.substr(i, pos - i);
        item_list.push_back(sub);
        data_obj.name = sub;
        item_map[sub] = data_obj;
        i = pos + 1;
    }

    pos = 0;
    std::string data_type(type);
    auto iter = item_list.begin();
    for (size_t i = 0; i < data_type.size(); ) {
        pos = data_type.find(',', i);
        if (pos >= std::string::npos) {
            std::string sub = data_type.substr(i, pos);
            item_map[*iter].type = sub;
            break;
        }
        std::string sub = data_type.substr(i, pos - i);
        item_map[*iter].type = sub;
        i = pos + 1;
        if (iter != item_list.end()) {
            iter++;
        }
    }

    DataRecipePtr result = std::make_shared<DataRecipe>();
    result->build(recipe_id, item_map, item_list);
    return result;
}

size_t Rtsi::socketRecv(void* out_msg, int size, ErrorCode* ec) {
    size_t rl = 0;
    rl = socket_ptr->receive(boost::asio::buffer(out_msg, size), MSG_WAITALL, *ec);
    return rl;
}

bool Rtsi::abortMessageRecv(int size) {
    boost::system::error_code ec;
    std::unique_ptr<uint8_t[]> temp_ptr(new uint8_t[size]);
    socketRecv(temp_ptr.get(), size, &ec);
    if (ec) {
        return false;
    }
    return true;
}

bool Rtsi::messageRecv(Command cmd, void* buff, int max_buff_size, ErrorCode* ec) {
    RtsiMsg::Head packet_head;
    if (max_buff_size < sizeof(RtsiMsg::Head)) {
        return false;
    }
    while (true) {
        if (socketRecv(&packet_head, sizeof(RtsiMsg::Head), ec) <= 0) {
            return false;
        }
        packet_head.size = htons(packet_head.size);
        if (packet_head.size <= 0) {
            ERROR_LOG("RTSI error packet head, will disconnect robot %d", packet_head.size);
            disconnect();
            break;
        }
        if (packet_head.type == cmd) {
            RtsiMsg::Head* packet = (RtsiMsg::Head*)buff;
            packet->size = packet_head.size;
            packet->type = packet_head.type;
            if (max_buff_size < packet->size) {
                ERROR_LOG(
                    "RTSI recv buff too small. "
                    "Target cmd: %d, target size %d, max buff size:%d, ",
                    packet->type,
                    packet->size,
                    max_buff_size);
                abortMessageRecv(packet_head.size - sizeof(RtsiMsg::Head));
                return false;
            }
            uint8_t* p_buff_data = (uint8_t*)buff + sizeof(RtsiMsg::Head);
            socketRecv(p_buff_data, (packet->size - sizeof(RtsiMsg::Head)), ec);
            return true;
        }
        else {
            abortMessageRecv(packet_head.size - sizeof(RtsiMsg::Head));
        }
    }
    return false;
}

bool Rtsi::messageSend(const void* msg, int max_msg_size, ErrorCode* ec) {
    RtsiMsg::Head* head = (RtsiMsg::Head*)msg;
    int msg_size = htons(head->size);
    if (msg_size > max_msg_size) {
        return false;
    }
    if (socket_ptr->send(boost::asio::buffer(head, msg_size), 0, *ec) <= 0) {
        return false;
    }
    return true;
}

void Rtsi::flipBytes(void* data, int size) {
    char* pdata = (char*)data;
    for (int i = 0; i < (size / 2); i++) {
        int j = size - i - 1;
        pdata[i] = pdata[i] ^ pdata[j];
        pdata[j] = pdata[i] ^ pdata[j];
        pdata[i] = pdata[i] ^ pdata[j];
    }
}

bool Rtsi::sendTextMessage(const std::string& message, const std::string& source, MessageType type) {
    uint16_t message_len = (uint16_t)(sizeof(RtsiMsg::Head) + sizeof(uint8_t) + message.length() + sizeof(uint8_t) + source.length() + sizeof(uint8_t));
    std::unique_ptr<uint8_t[]> buff = std::make_unique<uint8_t[]>(message_len);
    RtsiMsg::Head* msg = (RtsiMsg::Head*)buff.get();
    uint8_t* p_msg_data = buff.get() + sizeof(RtsiMsg::Head);
    size_t msg_offset = 0;
    msg->size = htons(message_len);
    msg->type = Rtsi::Command::TEXT_MESSAGE;
    if (message.length() >= UINT8_MAX) {
        return false;
    }
    p_msg_data[msg_offset] = (uint8_t)message.length();
    msg_offset++;
    memcpy(&p_msg_data[msg_offset], message.c_str(), message.length());
    msg_offset += message.length();

    if (source.length() >= UINT8_MAX) {
        return false;
    }
    p_msg_data[msg_offset] = (uint8_t)source.length();
    msg_offset++;
    memcpy(&p_msg_data[msg_offset], source.c_str(), source.length());
    msg_offset += source.length();

    p_msg_data[msg_offset] = (uint8_t)type;

    ErrorCode ec;
    return messageSend(msg, message_len, &ec);
}