#include "Rtsi.hpp"
#include "Logger.hpp"

using DataRecipe = Rtsi::DataRecipe;
using DataObject = Rtsi::DataObject;

DataObject& DataRecipe::operator[](const DataKey& name) {
    return valuemap[name];
}

bool DataRecipe::is_empty() {
    if (valuemap.empty() || itemlist.empty()) {
        return true;
    }
    return false;
}

const DataRecipe::ItemList& DataRecipe::getItemList() {
    return itemlist;
}

uint8_t DataRecipe::getID() {
    return id;
}

DataRecipe::DataRecipe() {

}

bool DataRecipe::build(uint8_t recipe_id, DataMap dmap, ItemList ilist) {
    valuemap = std::move(dmap);
    itemlist = std::move(ilist);
    id = recipe_id;
    return true;
}

DataRecipe::~DataRecipe() {
    valuemap.clear();
    itemlist.clear();
}

bool DataRecipe::convertToMessage(RtsiMsg::DataTxRx* msg_buff, int max_buff_size) {
    if (!msg_buff) {
        return false;
    }
    int count = 0;
    uint8_t* p_msg_data = ((uint8_t*)msg_buff) + sizeof(RtsiMsg::DataTxRx);
    for (auto item : itemlist) {
        DataObject& data_obj = valuemap[item];
        if (data_obj.type == "VECTOR6D") {
            memcpy(&p_msg_data[count], &data_obj.value.v6d[0], sizeof(DataObject::Data::v6d));
            for (size_t i = 0; i < 6; i++) {
                Rtsi::flipBytes(&p_msg_data[count + i * sizeof(double)], sizeof(double));
            }
            count += sizeof(DataObject::Data::v6d);
        
        } else if (data_obj.type == "VECTOR3D") {
            memcpy(&p_msg_data[count], &data_obj.value.v3d[0], sizeof(DataObject::Data::v3d));
            for (size_t i = 0; i < 3; i++) {
                Rtsi::flipBytes(&p_msg_data[count + i * sizeof(double)], sizeof(double));
            }
            count += sizeof(DataObject::Data::v3d);

        } else if (data_obj.type == "DOUBLE") {
            memcpy(&p_msg_data[count], &data_obj.value.d_64, sizeof(DataObject::Data::d_64));
            Rtsi::flipBytes(&p_msg_data[count], sizeof(double));
            count += sizeof(DataObject::Data::d_64);

        } else if (data_obj.type == "UINT32") {
            memcpy(&p_msg_data[count], &data_obj.value.u_32, sizeof(DataObject::Data::u_32));
            Rtsi::flipBytes(&p_msg_data[count], sizeof(uint32_t));
            count += sizeof(DataObject::Data::u_32);

        } else if (data_obj.type == "UINT64") {
            memcpy(&p_msg_data[count], &data_obj.value.u_64, sizeof(DataObject::Data::u_64));
            Rtsi::flipBytes(&p_msg_data[count], sizeof(uint64_t));
            count += sizeof(DataObject::Data::u_64);
           
        } else if (data_obj.type == "INT32") {
            memcpy(&p_msg_data[count], &data_obj.value.i_32, sizeof(DataObject::Data::i_32));
            Rtsi::flipBytes(&p_msg_data[count], sizeof(int32_t));
            count += sizeof(DataObject::Data::i_32);
           
        } else if (data_obj.type == "UINT8") {
            p_msg_data[count] = data_obj.value.u_8;
            count++;
           
        } else if (data_obj.type == "BOOL") {
            p_msg_data[count] = data_obj.value.b_8;
            count++;

        } else if (data_obj.type == "UINT16") {
            memcpy(&p_msg_data[count], &data_obj.value.u_16, sizeof(DataObject::Data::u_16));
            Rtsi::flipBytes(&p_msg_data[count], sizeof(uint16_t));
            count += sizeof(DataObject::Data::u_16);

        } else if (data_obj.type == "VECTOR6INT32") {
            memcpy(&p_msg_data[count], &data_obj.value.v6i32[0], sizeof(DataObject::Data::v6i32));
            for (size_t i = 0; i < 6; i++) {
                Rtsi::flipBytes(&p_msg_data[count + i * sizeof(int32_t)], sizeof(int32_t));
            }
            count += sizeof(DataObject::Data::v6i32);

        } else {
            ERROR_LOG("unknown data type: %d", valuemap[item].type);
        }
        if (count >= (max_buff_size - sizeof(RtsiMsg::DataTxRx))) {
            ERROR_LOG("Message buff is too small to save");
            return false;
        }
    }
    msg_buff->head.size = htons(count + (uint16_t)sizeof(RtsiMsg::DataTxRx));
    msg_buff->head.type = Command::DATA_PACKAGE;
    msg_buff->id = id;
    return true;
}

bool DataRecipe::setValueFromMessage(const RtsiMsg::DataTxRx& msg_buff) {
    int count = 0;
    uint8_t* p_msg_data = ((uint8_t*)&msg_buff) + sizeof(RtsiMsg::DataTxRx);
    for (auto item : itemlist) {
        DataObject& data_obj = valuemap[item];
        if (data_obj.type == "VECTOR6D") {
            memcpy(&data_obj.value.v6d[0], &p_msg_data[count], sizeof(DataObject::Data::v6d));
            for (size_t i = 0; i < 6; i++) {
                Rtsi::flipBytes(&data_obj.value.v6d[i], sizeof(double));
            }
            count += sizeof(DataObject::Data::v6d);
        
        } else if (data_obj.type == "VECTOR3D") {
            memcpy(&data_obj.value.v3d[0], &p_msg_data[count], sizeof(DataObject::Data::v3d));
            for (size_t i = 0; i < 3; i++) {
                Rtsi::flipBytes(&data_obj.value.v3d[i], sizeof(double));
            }
            count += sizeof(DataObject::Data::v3d);

        } else if (data_obj.type == "DOUBLE") {
            memcpy(&data_obj.value.d_64, &p_msg_data[count], sizeof(DataObject::Data::d_64));
            Rtsi::flipBytes(&data_obj.value.d_64, sizeof(double));
            count += sizeof(DataObject::Data::d_64);

        } else if (data_obj.type == "UINT32") {
            memcpy(&data_obj.value.u_32, &p_msg_data[count], sizeof(DataObject::Data::u_32));
            Rtsi::flipBytes(&data_obj.value.u_32, sizeof(uint32_t));
            count += sizeof(DataObject::Data::u_32);

        } else if (data_obj.type == "UINT64") {
            memcpy(&data_obj.value.u_64, &p_msg_data[count], sizeof(DataObject::Data::u_64));
            Rtsi::flipBytes(&data_obj.value.u_64, sizeof(uint64_t));
            count += sizeof(DataObject::Data::u_64);
           
        } else if (data_obj.type == "INT32") {
            memcpy(&data_obj.value.i_32, &p_msg_data[count], sizeof(DataObject::Data::i_32));
            Rtsi::flipBytes(&data_obj.value.i_32, sizeof(int32_t));
            count += sizeof(DataObject::Data::i_32);
           
        } else if (data_obj.type == "UINT8") {
            data_obj.value.u_8 = p_msg_data[count];
            count++;
           
        } else if (data_obj.type == "BOOL") {
            data_obj.value.b_8 = p_msg_data[count];
            count++;

        } else if (data_obj.type == "UINT16") {
            memcpy(&data_obj.value.u_16, &p_msg_data[count], sizeof(DataObject::Data::u_16));
            Rtsi::flipBytes(&data_obj.value.u_16, sizeof(uint16_t));
            count += sizeof(DataObject::Data::u_16);

        } else if (data_obj.type == "VECTOR6INT32") {
            memcpy(&data_obj.value.v6i32[0], &p_msg_data[count], sizeof(DataObject::Data::v6i32));
            for (size_t i = 0; i < 6; i++) {
                Rtsi::flipBytes(&data_obj.value.v6i32[i], sizeof(int32_t));
            }
            count += sizeof(DataObject::Data::v6i32);

        } else {
            ERROR_LOG("unknown data type: %d", valuemap[item].type);
        }
        if (count > (msg_buff.head.size - sizeof(RtsiMsg::DataTxRx))) {
            ERROR_LOG("Message buff is too small to save");
            return false;
        }
    }
    return true;
}


bool DataRecipe::has_not_found() {
    for (auto &item : valuemap) {
        if (item.second.type == "NOT_FOUND") {
            return true;
        }
    }
    return false;
}
    
bool DataRecipe::has_in_use() {
    for (auto &item : valuemap) {
        if (item.second.type == "IN_USE") {
            return true;
        }
    }
    return false;
}