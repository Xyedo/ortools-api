#ifndef ROUTING_H
#define ROUTING_H

// Include necessary standard or project-specific headers
#include <ortools/constraint_solver/constraint_solver.h>

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

// Namespace declarations (if needed)
namespace OrtoolsLib {
struct startEndPair {
  std::vector<int32_t> starts;
  std::vector<int32_t> ends;
};

struct SingleDepot {
  int32_t depot;
};

struct RoutingOptionWithCapacity {
  std::vector<int64_t> capacities;
  std::vector<int64_t> demands;
};
struct PickupDelivery {
  int64_t pickup;
  int64_t delivery;

  bool operator==(const PickupDelivery &b) const {
    return pickup == b.pickup && delivery == b.delivery;
  }
};
enum class PickupDropOption {
  FIFO,
  LIFO,
};
struct RoutingOptionWithPickupDelivery {
  std::optional<PickupDropOption> policy;
  std::vector<PickupDelivery> pickups_deliveries;
};

struct TimeWindow {
  int64_t start;
  int64_t end;

  bool operator<(const TimeWindow &b) const {
    if (start == b.start) {
      return end < b.end;
    }

    return start < b.start;
  }

  bool operator==(const TimeWindow &b) const {
    return start == b.start && end == b.end;
  }
};

struct RoutingOptionWithTimeWindow {
  std::vector<std::vector<TimeWindow>> time_windows;
};

struct RoutingOptionWithServiceTime {
  std::vector<int64_t> service_time;
};

struct RoutingOptionWithPenalties {
  // use global penalties to each node, or use penalties to each node
  // vector variant match one to one to node
  // if using vector variant, the depot location are ignored
  std::variant<std::vector<int64_t>, int64_t> penalties;
};

struct RoutingOptionWithVehicleBreakTime {
  std::vector<std::vector<TimeWindow>> break_time;
};

struct RoutingResponse {
  std::vector<int> route;
  int64_t total_duration;
};

class RoutingBuilder;
class Routing {
 private:
  std::vector<std::vector<int64_t>> _duration_matrix;
  std::variant<SingleDepot, startEndPair> _depot_config;
  int32_t _num_vehicles = 1;
  std::optional<int64_t> _time_limit;
  std::optional<RoutingOptionWithCapacity> _with_capacity;
  std::optional<RoutingOptionWithPickupDelivery> _with_pickup_delivery;
  std::optional<RoutingOptionWithTimeWindow> _with_time_window;
  std::optional<RoutingOptionWithServiceTime> _with_service_time;
  std::optional<RoutingOptionWithPenalties> _with_drop_penalties;
  std::optional<RoutingOptionWithVehicleBreakTime> _with_vehicle_break_time;
  Routing() {};
  void _addTimeWindow(operations_research::IntVar *const time_dimension,
                      std::vector<TimeWindow> &time_window);
  void _addDummyLocAtEnd();
  void _duplicateNodesToBack(int at);

 public:
  Routing(const Routing &other)
      : _duration_matrix(other._duration_matrix),
        _depot_config(other._depot_config),
        _num_vehicles(other._num_vehicles),
        _time_limit(other._time_limit),
        _with_capacity(other._with_capacity),
        _with_pickup_delivery(other._with_pickup_delivery),
        _with_time_window(other._with_time_window),
        _with_service_time(other._with_service_time),
        _with_drop_penalties(other._with_drop_penalties),
        _with_vehicle_break_time(other._with_vehicle_break_time) {}

  Routing &operator=(const Routing &other) { return *this = Routing(other); }
  Routing(Routing &&other) noexcept
      : _duration_matrix(std::move(other._duration_matrix)),
        _depot_config(std::move(other._depot_config)),
        _num_vehicles(other._num_vehicles),
        _time_limit(other._time_limit),
        _with_capacity(std::move(other._with_capacity)),
        _with_pickup_delivery(std::move(other._with_pickup_delivery)),
        _with_time_window(std::move(other._with_time_window)),
        _with_service_time(std::move(other._with_service_time)),
        _with_drop_penalties(std::move(other._with_drop_penalties)),
        _with_vehicle_break_time(std::move(other._with_vehicle_break_time)) {}

  Routing &operator=(Routing &&other) noexcept {
    return *this = Routing(other);
  }

  static RoutingBuilder builder();
  friend class RoutingBuilder;
  std::vector<RoutingResponse> solve();
};

class RoutingBuilder {
 private:
  Routing _routing;
  void _validate();

 public:
  RoutingBuilder(Routing &r) : _routing(r) {}
  RoutingBuilder &setDurationMatrix(
      const std::vector<std::vector<int64_t>> matrix) {
    _routing._duration_matrix = matrix;
    return *this;
  }
  RoutingBuilder &setDepotConfig(
      const std::variant<SingleDepot, startEndPair> depot) {
    _routing._depot_config = depot;
    return *this;
  }
  RoutingBuilder &setNumVehicles(const int32_t num_vehicles) {
    _routing._num_vehicles = num_vehicles;
    return *this;
  }

  RoutingBuilder &setTimeLimit(const int64_t time_limit) {
    _routing._time_limit = time_limit;
    return *this;
  }
  RoutingBuilder &withCapacity(
      const std::optional<RoutingOptionWithCapacity> with_capacity) {
    _routing._with_capacity = with_capacity;
    return *this;
  }

  RoutingBuilder &withPickupDelivery(
      const std::optional<RoutingOptionWithPickupDelivery>
          with_pickup_delivery) {
    _routing._with_pickup_delivery = with_pickup_delivery;
    return *this;
  }
  RoutingBuilder &withTimeWindow(
      const std::optional<RoutingOptionWithTimeWindow> with_time_window) {
    _routing._with_time_window = with_time_window;
    return *this;
  }
  RoutingBuilder &withServiceTime(
      const std::optional<RoutingOptionWithServiceTime> with_service_time) {
    _routing._with_service_time = with_service_time;
    return *this;
  }
  RoutingBuilder &withDropPenalties(
      const std::optional<RoutingOptionWithPenalties> with_drop_penalties) {
    _routing._with_drop_penalties = with_drop_penalties;
    return *this;
  }
  RoutingBuilder &withVehicleBreakTime(
      const std::optional<RoutingOptionWithVehicleBreakTime>
          with_vehicle_break_time) {
    _routing._with_vehicle_break_time = with_vehicle_break_time;
    return *this;
  }

  Routing build();
};

}  // namespace OrtoolsLib

#endif  // ROUTING_H