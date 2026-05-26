#include "motion_core/config/axis_config_json.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

namespace motion_core {

// ADL functions MUST be in the same namespace as the types (motion_core)
// and NOT in anonymous namespace to be found by nlohmann::json.

void to_json(json& j, const AxisConfigParameterMetadata& meta) {
    j = json{
        {"domain_name", meta.domain_name},
        {"name", meta.name},
        {"group", meta.group},
        {"unit", meta.unit},
        {"read_only", meta.read_only},
        {"persistable", meta.persistable}
    };
}

void from_json(const json& j, AxisConfigParameterMetadata& meta) {
    meta.domain_name = j.value("domain_name", std::string{});
    meta.name = j.value("name", std::string{});
    meta.group = j.value("group", std::string{});
    meta.unit = j.value("unit", std::string{});
    meta.read_only = j.value("read_only", false);
    meta.persistable = j.value("persistable", false);
}

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

void to_json(json& j, const AxisConfigParameterRecord& record) {
    j = json{
        {"d", static_cast<int>(record.entry.id.domain)},
        {"i", record.entry.id.value},
        {"v", record.entry.value},
        {"meta", record.meta}
    };
}

void from_json(const json& j, AxisConfigParameterRecord& record) {
    record.entry.id.domain = static_cast<ParameterDomain>(j.at("d").get<int>());
    record.entry.id.value = j.at("i").get<uint32_t>();
    record.entry.value = j.at("v").get<ParameterValue>();
    if (j.contains("meta")) {
        record.meta = j.at("meta").get<AxisConfigParameterMetadata>();
    }
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
        cfg.gear_ratio = j.value("gear_ratio", 1.0);
        cfg.encoder_resolution_bits = j.value("enc_res", 0U);
        
        if (j.contains("params")) {
            const auto& params = j.at("params");
            if (!params.is_array()) {
                return Result<AxisConfig>::failure({ErrorCode::ProtocolFailure, "axis config params must be an array"});
            }

            for (const auto& item : params) {
                if (item.is_object() && item.contains("meta")) {
                    auto record = item.get<AxisConfigParameterRecord>();
                    cfg.parameters.entries.push_back(record.entry);
                    cfg.parameter_records.push_back(std::move(record));
                    continue;
                }

                auto entry = item.get<ParameterEntry>();
                cfg.parameters.entries.push_back(entry);
                cfg.parameter_records.push_back(AxisConfigParameterRecord{entry, {}});
            }
        }
        
        return Result<AxisConfig>::success(std::move(cfg));
    } catch (const std::exception& e) {
        return Result<AxisConfig>::failure({ErrorCode::ProtocolFailure, "json parse error"});
    }
}

Result<void> save_axis_config_to_file(const std::string& path, const AxisConfig& config) {
    try {
        json j;
        j["version"] = std::max<std::uint32_t>(config.version, 2U);
        j["axis_id"] = config.axis_id.value;
        j["axis_name"] = config.axis_name.value;
        j["transport"] = static_cast<int>(config.transport);
        j["gear_ratio"] = config.gear_ratio;
        j["enc_res"] = config.encoder_resolution_bits;

        if (!config.parameter_records.empty()) {
            j["params"] = config.parameter_records;
        } else {
            std::vector<AxisConfigParameterRecord> records;
            records.reserve(config.parameters.entries.size());
            for (const auto& entry : config.parameters.entries) {
                records.push_back(AxisConfigParameterRecord{entry, {}});
            }
            j["params"] = records;
        }

        const std::filesystem::path output_path(path);
        const auto parent_dir = output_path.parent_path();
        if (!parent_dir.empty()) {
            std::error_code ec;
            std::filesystem::create_directories(parent_dir, ec);
            if (ec) {
                return Result<void>::failure(
                    {ErrorCode::InternalError,
                     "failed to create axis config directory"});
            }
        }

        std::ofstream f(output_path);
        if (!f.is_open()) {
            return Result<void>::failure(
                {ErrorCode::InternalError,
                 "failed to write axis config file"});
        }
        f << j.dump(4);
        return Result<void>::success();
    } catch (...) {
        return Result<void>::failure({ErrorCode::InternalError, "failed to serialize axis config"});
    }
}

} // namespace motion_core
