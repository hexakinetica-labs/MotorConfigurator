#include "motion_core/config/axis_config_json.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

namespace motion_core {

// ADL functions MUST be in the same namespace as the types (motion_core)
// and NOT in anonymous namespace to be found by nlohmann::json.

void to_json(json& j, const ParameterValue& v) {
    j = json::object();
    j["type"] = static_cast<int>(v.type);
    switch (v.type) {
        case ParameterValueType::SignedInteger: j["v"] = v.signed_value; break;
        case ParameterValueType::UnsignedInteger: j["v"] = v.unsigned_value; break;
        case ParameterValueType::FloatingPoint: j["v"] = v.floating_value; break;
        case ParameterValueType::Boolean: j["v"] = v.bool_value; break;
    }
}

void from_json(const json& j, ParameterValue& v) {
    int type_int = j.at("type").get<int>();
    v.type = static_cast<ParameterValueType>(type_int);
    switch (v.type) {
        case ParameterValueType::SignedInteger: v.signed_value = j.at("v").get<int64_t>(); break;
        case ParameterValueType::UnsignedInteger: v.unsigned_value = j.at("v").get<uint64_t>(); break;
        case ParameterValueType::FloatingPoint: v.floating_value = j.at("v").get<double>(); break;
        case ParameterValueType::Boolean: v.bool_value = j.at("v").get<bool>(); break;
    }
}

void to_json(json& j, const ParameterEntry& e) {
    j = json{
        {"d", static_cast<int>(e.id.domain)},
        {"i", e.id.value},
        {"v", e.value}
    };
}

void from_json(const json& j, ParameterEntry& e) {
    e.id.domain = static_cast<ParameterDomain>(j.at("d").get<int>());
    e.id.value = j.at("i").get<uint32_t>();
    e.value = j.at("v").get<ParameterValue>();
}

Result<AxisConfig> load_axis_config_from_file(const std::string& path) {
    try {
        std::ifstream f(path);
        if (!f.is_open()) {
            return Result<AxisConfig>::failure({ErrorCode::NotFound, "failed to open axis config file"});
        }
        
        json j;
        f >> j;
        
        AxisConfig cfg;
        cfg.version = j.at("version").get<uint32_t>();
        cfg.axis_id.value = j.at("axis_id").get<uint16_t>();
        cfg.axis_name.value = j.at("axis_name").get<std::string>();
        cfg.transport = static_cast<AxisTransportKind>(j.at("transport").get<int>());
        cfg.gear_ratio = j.at("gear_ratio").get<double>();
        cfg.encoder_resolution_bits = j.at("enc_res").get<uint32_t>();
        
        if (j.contains("params")) {
            cfg.parameters.entries = j.at("params").get<std::vector<ParameterEntry>>();
        }
        
        return Result<AxisConfig>::success(std::move(cfg));
    } catch (const std::exception& e) {
        return Result<AxisConfig>::failure({ErrorCode::ProtocolFailure, "json parse error"});
    }
}

Result<void> save_axis_config_to_file(const std::string& path, const AxisConfig& config) {
    try {
        json j;
        j["version"] = config.version;
        j["axis_id"] = config.axis_id.value;
        j["axis_name"] = config.axis_name.value;
        j["transport"] = static_cast<int>(config.transport);
        j["gear_ratio"] = config.gear_ratio;
        j["enc_res"] = config.encoder_resolution_bits;
        j["params"] = config.parameters.entries;
        
        std::ofstream f(path);
        if (!f.is_open()) {
            return Result<void>::failure({ErrorCode::InternalError, "failed to write axis config file"});
        }
        f << j.dump(4);
        return Result<void>::success();
    } catch (...) {
        return Result<void>::failure({ErrorCode::InternalError, "failed to serialize axis config"});
    }
}

} // namespace motion_core
