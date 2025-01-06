#include "RobotState.hpp"
#include <fstream>
#include <iostream>
#include <regex>

struct DataType {
    const char *type;
    int size;
};

static constexpr DataType s_data_type_list[] = {
    {"uint64_t", sizeof(uint64_t)}, {"int64_t", sizeof(int64_t)}, {"uint32_t", sizeof(uint32_t)}, {"int32_t", sizeof(int32_t)},
    {"uint16_t", sizeof(uint16_t)}, {"int16_t", sizeof(int16_t)}, {"uint8_t", sizeof(uint8_t)},   {"int8_t", sizeof(int8_t)},
    {"bool", sizeof(bool)},         {"double", sizeof(double)},   {"float", sizeof(float)}};

static constexpr int maxDataTypeSize() {
    constexpr size_t data_type_list_size = sizeof(s_data_type_list) / sizeof(DataType);
    int max = 0;
    for (size_t i = 0; i < data_type_list_size; i++) {
        if (max < s_data_type_list[i].size) {
            max = s_data_type_list[i].size;
        }
    }
    return max;
}

static constexpr size_t dataTypeListLen() { return (sizeof(s_data_type_list) / sizeof(DataType)); }

bool RobotState::LoadFile::insertItem(const RobotState::Item &item, RobotState::Packages *packages,
                                      RobotState::PackagesOrder *packages_order) {
    if (item.name.find("reserve") == std::string::npos) {
        if (packages->find(item.name) != packages->end()) {
            std::cout << "Load file fail: has same item " << item.name << std::endl;
            return false;
        }
    } else {
        if (packages->find(item.name) != packages->end()) {
            if (packages->at(item.name).size != item.size) {
                std::cout << "Load file fail: has different reserve size " << item.name << ": " << item.size
                          << " != " << packages->at(item.name).size << std::endl;
                return false;
            }
        }
    }

    packages->insert(std::make_pair(item.name, item));
    packages_order->push_back(item.name);
    return true;
}

bool RobotState::LoadFile::loadFile(const std::string &file_path, RobotState::Packages *packages,
                                    RobotState::PackagesOrder *packages_order) {
    std::vector<RobotState::Item> foreach_pack;
    unsigned int message_size = 0;
    bool is_foreach = false;
    std::ifstream ifs;
    ifs.open(file_path, std::ios_base::in);
    if (!ifs.is_open()) {
        return false;
    }
    packages->clear();
    packages_order->clear();
    std::regex re("sub-package\\([0-9]+ bytes\\)");
    std::smatch match;
    std::string prefix;
    for (std::string line; std::getline(ifs, line);) {
        if (std::regex_search(line, match, re)) {
            prefix.clear();
            size_t pos = line.find(match.str(0));
            if (pos != std::string::npos) {
                prefix = line.substr(0, pos);
                std::string prefix_repalce;
                std::regex_replace(std::back_inserter(prefix_repalce), prefix.begin(), prefix.end(), std::regex(" "), "_");
                prefix = std::move(prefix_repalce);
            }
        }
        if (line.find("foreach joint:") != std::string::npos) {
            foreach_pack.clear();
            is_foreach = true;
        }
        if (line.find("\tend") != std::string::npos) {
            for (size_t i = 0; i < 6; i++) {
                for (auto each : foreach_pack) {
                    each.name += std::to_string(i);
                    if (!insertItem(each, packages, packages_order)) {
                        return false;
                    }
                    message_size += each.size;
                }
            }
            is_foreach = false;
        }
        RobotState::Item item;
        item.name = "";
        item.size = 0;
        memset(item.data_buff, 0, sizeof(item.data_buff));
        if (analyzeItem(line, &item, prefix)) {
            if (is_foreach) {
                foreach_pack.push_back(item);
            } else {
                if (!insertItem(item, packages, packages_order)) {
                    return false;
                }
                message_size += item.size;
            }
        }
    }

    ifs.close();
    return true;
}

bool RobotState::LoadFile::analyzeItem(const std::string &line, RobotState::Item *item, std::string &prefix) {
    for (size_t i = 0; i < dataTypeListLen(); i++) {
        std::size_t pos = line.find(s_data_type_list[i].type);
        if (pos == std::string::npos) {
            continue;
        }
        item->name.clear();
        item->name = prefix;
        const char *name_str = line.c_str() + pos + strlen(s_data_type_list[i].type) + 1;
        for (; *name_str != '\0'; name_str++) {
            if (*name_str == '\t' || *name_str == ' ') {
                break;
            }
            item->name += (*name_str);
        }

        item->size = s_data_type_list[i].size;
        return true;
    }
    return false;
}