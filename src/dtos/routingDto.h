#ifndef routingDto_h
#define routingDto_h

#include <lib/routing.h>
#include <routing-proto/routing.grpc.pb.h>

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>
#include <json/json.h>

namespace RoutingDTO {
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

RoutingModel intoEntity(const routing::RoutingRequest *const request);
RoutingModel parseJSON(std::shared_ptr<Json::Value> json);
}  // namespace RoutingDTO

#endif