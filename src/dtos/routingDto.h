#ifndef routingDto_h
#define routingDto_h

#include <lib/routing.h>
#include <routing-proto/routing.grpc.pb.h>

#include <cstdint>
#include <optional>
#include <variant>
#include <json/json.h>
#include <exception>

namespace RoutingDTO {

class ParseErrorElement: public std::exception {
    std::string code = "PARSE_ERROR";
    std::string key;
    std::optional<std::vector<std::string>> values;

public:
    ParseErrorElement(std::string key) : key(key), values(std::nullopt) {}
    ParseErrorElement(std::string key, std::vector<std::string> values) : key(key), values(values) {}

    Json::Value toJson() const noexcept {
        Json::Value json;
        json["code"] = code;

        if (!values.has_value()) {
            json["errors"] = key;

            return json;
        }
        json["errors"] = "invalid payload";
        Json::Value data;


        data["key"] = key;
        if (values.has_value()) {
            data["values"] = Json::Value(Json::arrayValue);
            for (const auto &value : values.value()) {
                data["values"].append(value);
            }
        }

        json["data"] = data;
        return json;
    }

    const char* what() const noexcept override {
        return "ParseErrorElement";
    }
};


struct RoutingModel {
  std::vector<std::vector<int64_t>> duration_matrix;
  std::variant<OrtoolsLib::SingleDepot, OrtoolsLib::startEndPair> depot_config;
  int32_t num_vehicles = 1;
  int64_t time_limit;
  std::optional<OrtoolsLib::RoutingOptionWithCapacity> with_capacity;
  std::optional<OrtoolsLib::RoutingOptionWithPickupDelivery>
      with_pickup_delivery;
  std::optional<OrtoolsLib::RoutingOptionWithTimeWindow> with_time_window;
  std::optional<OrtoolsLib::RoutingOptionWithServiceTime> with_service_time;
  std::optional<OrtoolsLib::RoutingOptionWithPenalties> with_drop_penalties;
  std::optional<OrtoolsLib::RoutingOptionWithVehicleBreakTime>
      with_vehicle_break_time;
};

RoutingModel intoEntity(const routing::RoutingRequest *const request) noexcept;
RoutingModel parseJSON(std::shared_ptr<Json::Value> json);
}  // namespace RoutingDTO

#endif
