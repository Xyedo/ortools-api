#ifndef ROUTING_H
#define ROUTING_H

using namespace std;
// Include necessary standard or project-specific headers
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

// Namespace declarations (if needed)
namespace ortoolsRouting {
struct startEndPair {
  const vector<int32_t> starts;
  const vector<int32_t> ends;
};

struct SingleDepot {
  int32_t depot;
};

struct RoutingOptionWithCapacity {
  vector<int64_t> capacities;
  vector<int64_t> demands;
};
struct PickupDelivery {
  int32_t pickup;
  int32_t delivery;
};

struct RoutingOptionWithPickupDelivery {
  vector<PickupDelivery> pickups_deliveries;
};

struct TimeWindow {
  int64_t start;
  int64_t end;
};

struct RoutingOptionWithTimeWindow {
  vector<vector<TimeWindow>> time_windows;
};

struct RoutingOptionWithServiceTime {
  vector<int64_t> service_time;
};

struct RoutingOptionWithPenalties {
  // use global penalties to each node, or use penalties to each node
  // vector variant match one to one to node
  // if using vector variant, the depot location are ignored
  variant<vector<int64_t>, int64_t> penalties;
};

struct RoutingOptionWithVehicleBreakTime {
  const vector<vector<TimeWindow>> break_time;
};

struct RoutingConfig {
  vector<vector<int64_t>> duration_matrix;
  const variant<SingleDepot, startEndPair> depot_config;
  const int32_t num_vehicles = 1;
  const optional<int64_t> time_limit;
  optional<RoutingOptionWithCapacity> with_capacity;
  optional<RoutingOptionWithPickupDelivery> with_pickup_delivery;
  optional<RoutingOptionWithTimeWindow> with_time_window;
  optional<RoutingOptionWithServiceTime> with_service_time;
  optional<RoutingOptionWithPenalties> with_drop_penalties;
  const optional<RoutingOptionWithVehicleBreakTime> with_vehicle_break_time;
};

struct RoutingResponse {
  vector<int> route;
  int64_t total_duration;
};

// Function declarations
vector<RoutingResponse> Routing(RoutingConfig config);

}  // namespace ortoolsRouting

#endif  // ROUTING_H