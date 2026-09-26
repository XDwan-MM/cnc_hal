/* 将 KickCAT 的 ESI 模型投影为本项目的离线 JSON 描述；不连接主站。 */
#include <kickcat/ESI/Parser.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

void quoted(const std::string& value) {
    std::cout << '"';
    constexpr char hex[] = "0123456789abcdef";
    for (unsigned char c : value) {
        switch (c) {
        case '"': std::cout << "\\\""; break;
        case '\\': std::cout << "\\\\"; break;
        case '\n': std::cout << "\\n"; break;
        case '\r': std::cout << "\\r"; break;
        case '\t': std::cout << "\\t"; break;
        default:
            if (c < 0x20) {
                std::cout << "\\u00" << hex[c >> 4] << hex[c & 15];
            } else {
                std::cout << static_cast<char>(c);
            }
        }
    }
    std::cout << '"';
}

void pdo_list(const std::vector<kickcat::ESI::Pdo>& pdos) {
    std::cout << '[';
    bool first_pdo = true;
    for (const auto& pdo : pdos) {
        if (!first_pdo) std::cout << ',';
        first_pdo = false;
        std::cout << "{\"index\":" << pdo.index << ",\"name\":";
        quoted(pdo.name);
        std::cout << ",\"sm\":";
        if (pdo.sm) std::cout << *pdo.sm;
        else std::cout << "null";
        std::cout << ",\"su\":";
        if (pdo.su) std::cout << *pdo.su;
        else std::cout << "null";
        std::cout << ",\"fixed\":" << (pdo.fixed ? "true" : "false")
                  << ",\"mandatory\":" << (pdo.mandatory ? "true" : "false")
                  << ",\"exclude\":[";
        for (size_t i = 0; i < pdo.exclude.size(); ++i) {
            if (i) std::cout << ',';
            std::cout << pdo.exclude[i];
        }
        std::cout << "],\"excluded_sm\":[";
        for (size_t i = 0; i < pdo.excluded_sm.size(); ++i) {
            if (i) std::cout << ',';
            std::cout << pdo.excluded_sm[i];
        }
        std::cout << "],\"entries\":[";
        bool first_entry = true;
        for (const auto& entry : pdo.entries) {
            if (!first_entry) std::cout << ',';
            first_entry = false;
            std::cout << "{\"index\":" << entry.index
                      << ",\"subindex\":" << static_cast<unsigned>(entry.subindex)
                      << ",\"bits\":" << entry.bit_len
                      << ",\"fixed\":" << (entry.fixed ? "true" : "false")
                      << ",\"name\":";
            quoted(entry.name);
            std::cout << ",\"data_type\":";
            quoted(entry.data_type);
            std::cout << '}';
        }
        std::cout << "]}";
    }
    std::cout << ']';
}

void device(const kickcat::ESI::Device& dev) {
    std::cout << "{\"vendor_id\":" << dev.vendor_id
              << ",\"product_code\":" << dev.product_code
              << ",\"revision\":" << dev.revision_no << ",\"type\":";
    quoted(dev.type);
    std::cout << ",\"name\":";
    quoted(dev.name);
    std::cout << ",\"vendor_name\":";
    quoted(dev.vendor_name);
    std::cout << ",\"sm\":[";
    for (size_t i = 0; i < dev.sync_managers.size(); ++i) {
        if (i) std::cout << ',';
        const auto& sm = dev.sync_managers[i];
        std::cout << "{\"index\":" << i << ",\"type\":" << static_cast<unsigned>(sm.type)
                  << ",\"enabled\":" << (sm.enable ? "true" : "false")
                  << ",\"start_address\":" << sm.start_address
                  << ",\"min_size\":" << sm.min_size
                  << ",\"max_size\":" << sm.max_size
                  << ",\"default_size\":" << sm.default_size << '}';
    }
    std::cout << "],\"coe\":";
    if (dev.mailbox && dev.mailbox->coe) {
        const auto& coe = *dev.mailbox->coe;
        std::cout << "{\"pdo_assign\":" << (coe.pdo_assign ? "true" : "false")
                  << ",\"pdo_config\":" << (coe.pdo_config ? "true" : "false")
                  << ",\"pdo_upload\":" << (coe.pdo_upload ? "true" : "false")
                  << ",\"sdo_info\":" << (coe.sdo_info ? "true" : "false") << '}';
    } else std::cout << "null";
    std::cout << ",\"rx_pdos\":";
    pdo_list(dev.rx_pdos);
    std::cout << ",\"tx_pdos\":";
    pdo_list(dev.tx_pdos);
    /* KickCAT 会根据 PDO 合成缺失的字典条目；调用方不得把这些权限当作设备实测权限。 */
    std::cout << ",\"objects_may_be_synthesized\":true,\"objects\":[";
    bool first_object = true;
    for (const auto& object : dev.dictionary) {
        if (!first_object) std::cout << ',';
        first_object = false;
        std::cout << "{\"index\":" << object.index << ",\"name\":";
        quoted(object.name);
        std::cout << ",\"entries\":[";
        bool first_entry = true;
        for (const auto& entry : object.entries) {
            if (!first_entry) std::cout << ',';
            first_entry = false;
            std::cout << "{\"subindex\":" << static_cast<unsigned>(entry.subindex)
                      << ",\"bits\":" << entry.bitlen
                      << ",\"type\":" << static_cast<unsigned>(entry.type)
                      << ",\"access\":" << entry.access << '}';
        }
        std::cout << "]}";
    }
    std::cout << "],\"dc_modes\":[";
    if (dev.dc) {
        for (size_t i = 0; i < dev.dc->op_modes.size(); ++i) {
            if (i) std::cout << ',';
            const auto& mode = dev.dc->op_modes[i];
            std::cout << "{\"name\":";
            quoted(mode.name);
            std::cout << ",\"assign_activate\":" << mode.assign_activate << '}';
        }
    }
    std::cout << "]}";
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "用法: esi_extract <设备ESI.xml>\n";
        return 2;
    }
    try {
        kickcat::ESI::Parser parser;
        std::vector<std::string> errors;
        const auto devices = parser.loadAllDevices(argv[1], &errors);
        if (!errors.empty() || devices.empty()) {
            for (const auto& error : errors) std::cerr << error << '\n';
            if (devices.empty()) std::cerr << "ESI 没有可解析的设备\n";
            return 1;
        }
        std::cout << "{\"schema_version\":1,\"source\":";
        quoted(argv[1]);
        std::cout << ",\"devices\":[";
        for (size_t i = 0; i < devices.size(); ++i) {
            if (i) std::cout << ',';
            device(devices[i]);
        }
        std::cout << "]}\n";
        return std::cout.good() ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "ESI 解析失败: " << error.what() << '\n';
        return 1;
    }
}
