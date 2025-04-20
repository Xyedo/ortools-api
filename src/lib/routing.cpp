

#include "routing.h"

#include <ortools/constraint_solver/constraint_solver.h>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <variant>
#include <vector>

namespace OrtoolsLib {
std::vector<RoutingResponse> Routing::solve() {
  std::map<int, int> new_index_to_old_index;
  std::set<int> pick_drop_set;
  if (_with_pickup_delivery.has_value()) {
    for (auto &pair : _with_pickup_delivery.value().pickups_deliveries) {
      if (pick_drop_set.count(pair.pickup)) {
        _duplicateNodesToBack(pair.pickup);
        new_index_to_old_index[_duration_matrix.size() - 1] = pair.pickup;
        pair.pickup = _duration_matrix.size() - 1;
      } else {
        pick_drop_set.insert(pair.pickup);
      }

      if (pick_drop_set.count(pair.delivery)) {
        _duplicateNodesToBack(pair.delivery);
        new_index_to_old_index[_duration_matrix.size() - 1] = pair.delivery;
        pair.delivery = _duration_matrix.size() - 1;
      } else {
        pick_drop_set.insert(pair.delivery);
      }
    }
  }

  std::optional<operations_research::RoutingIndexManager> optManager;
  const SingleDepot *depot = std::get_if<SingleDepot>(&_depot_config);
  const startEndPair *start_end = std::get_if<startEndPair>(&_depot_config);
  if (depot) {
    auto m_depot = depot->depot;
    if (m_depot == -1) {
      _addDummyLocAtEnd();
      m_depot = _duration_matrix.size() - 1;
    }

    if (pick_drop_set.count(m_depot)) {
      _duplicateNodesToBack(m_depot);
      new_index_to_old_index[_duration_matrix.size() - 1] = m_depot;
      m_depot = _duration_matrix.size() - 1;
    }

    optManager.emplace(_duration_matrix.size(), _num_vehicles,
                       operations_research::RoutingNodeIndex{m_depot});
  } else if (start_end) {
    std::vector<operations_research::RoutingNodeIndex> m_start_nodes(
        start_end->starts.begin(), start_end->starts.end());
    std::vector<operations_research::RoutingNodeIndex> m_end_nodes(
        start_end->ends.begin(), start_end->ends.end());

    if (find(m_start_nodes.begin(), m_start_nodes.end(), -1) !=
            m_start_nodes.end() ||
        find(m_end_nodes.begin(), m_end_nodes.end(), -1) != m_end_nodes.end()) {
      _addDummyLocAtEnd();
      for (auto &start : m_start_nodes) {
        if (start == -1) {
          start = _duration_matrix.size() - 1;
        }
      }
      for (auto &end : m_end_nodes) {
        if (end == -1) {
          end = _duration_matrix.size() - 1;
        }
      }
    }

    for (auto &start : m_start_nodes) {
      if (pick_drop_set.count(start.value())) {
        _duplicateNodesToBack(start.value());
        new_index_to_old_index[_duration_matrix.size() - 1] = start.value();
        start = _duration_matrix.size() - 1;
      }
    }

    for (auto &end : m_end_nodes) {
      if (pick_drop_set.count(end.value())) {
        _duplicateNodesToBack(end.value());
        new_index_to_old_index[_duration_matrix.size() - 1] = end.value();
        end = _duration_matrix.size() - 1;
      }
    }

    optManager.emplace(_duration_matrix.size(), _num_vehicles, m_start_nodes,
                       m_end_nodes);
  } else {
    throw std::invalid_argument("Invalid depot configuration");
  }

  operations_research::RoutingIndexManager manager = optManager.value();

  operations_research::RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [this, &manager](int64_t from_index, int64_t to_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();

        if (_with_service_time.has_value()) {
          const int64_t service_time =
              _with_service_time.value().service_time[from_node];
          return _duration_matrix[from_node][to_node] + service_time;
        }

        return _duration_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  const std::string time = "Time";

  int64_t time_capacity = INT64_MAX;
  if (_with_time_window.has_value()) {
    auto &twss = _with_time_window.value().time_windows;
    int64_t mx = 0;

    for (const auto &tws : twss) {
      for (const auto &tw : tws) {
        if (tw.start == 0 && tw.end == INT64_MAX) {
          continue;
        }

        mx = std::max(mx, tw.start);
        mx = std::max(mx, tw.end);
      }
    }
    if (mx > 0) {
      time_capacity = mx;
    }
  }

  int64_t slack_time = 0;
  if (_with_vehicle_break_time.has_value()) {
    int64_t mx = 0;
    const auto &break_times = _with_vehicle_break_time.value().break_time;
    for (const auto &break_time : break_times) {
      for (const auto &bt : break_time) {
        mx = std::max(mx, bt.end - bt.start);
      }
    }

    slack_time = mx;
  }
  routing.AddDimension(transit_callback_index, slack_time, time_capacity,
                       !_with_time_window.has_value(), time);

  operations_research::RoutingDimension &time_dimension =
      *routing.GetMutableDimension(time);

  if (_with_capacity.has_value()) {
    const std::vector<int64_t> &capacities = _with_capacity.value().capacities;
    const std::vector<int64_t> &demands = _with_capacity.value().demands;
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [&demands, &manager](const int64_t from_index) -> int64_t {
          const int from_node = manager.IndexToNode(from_index).value();
          return demands[from_node];
        });
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,  // transit callback index
        int64_t{0},             // null capacity slack
        capacities,             // vehicle maximum capacities
        true,                   // start cumul to zero
        "Capacity");
  }

  if (_with_pickup_delivery.has_value()) {
    const auto &pd_config = _with_pickup_delivery.value();
    const auto &pickups_deliveries = pd_config.pickups_deliveries;
    const auto &pd_policy = pd_config.policy;

    operations_research::Solver *const solver = routing.solver();
    for (const PickupDelivery &pair : pickups_deliveries) {
      const int64_t pickup_index = manager.NodeToIndex(
          operations_research::RoutingIndexManager::NodeIndex(pair.pickup));
      const int64_t delivery_index = manager.NodeToIndex(
          operations_research::RoutingIndexManager::NodeIndex(pair.delivery));
      routing.AddPickupAndDelivery(pickup_index, delivery_index);
      solver->AddConstraint(
          solver->MakeEquality(static_cast<operations_research::IntVar *>(
                                   routing.VehicleVar(pickup_index)),
                               static_cast<operations_research::IntVar *>(
                                   routing.VehicleVar(delivery_index))));
      solver->AddConstraint(
          solver->MakeLessOrEqual(time_dimension.CumulVar(pickup_index),
                                  time_dimension.CumulVar(delivery_index)));
    }

    if (pd_policy.has_value()) {
      switch (pd_policy.value()) {
        case PickupDropOption::FIFO:
          routing.SetPickupAndDeliveryPolicyOfAllVehicles(
              operations_research::RoutingModel::PICKUP_AND_DELIVERY_FIFO);
          break;
        case PickupDropOption::LIFO:
          routing.SetPickupAndDeliveryPolicyOfAllVehicles(
              operations_research::RoutingModel::PICKUP_AND_DELIVERY_LIFO);
          break;
        default:
          throw std::invalid_argument("Invalid pickup and delivery policy");
      }
    }
  }

  if (_with_time_window.has_value()) {
    std::vector<std::vector<TimeWindow>> &time_windows =
        _with_time_window.value().time_windows;
    for (int i = 0; i < time_windows.size(); ++i) {
      std::sort(time_windows[i].begin(), time_windows[i].end());

      if (depot && depot->depot == i) {
        continue;
      }

      if (start_end) {
        const std::vector<int> &starts = start_end->starts;
        const std::vector<int> &ends = start_end->ends;
        if (std::find(starts.begin(), starts.end(), i) != starts.end()) {
          continue;
        }
        if (std::find(ends.begin(), ends.end(), i) != ends.end()) {
          continue;
        }
      }

      _addTimeWindow(
          time_dimension.CumulVar(manager.NodeToIndex(
              operations_research::RoutingIndexManager::NodeIndex(i))),
          time_windows[i]);
    }

    for (int i = 0; i < _num_vehicles; ++i) {
      const int64_t route_start_idx = routing.Start(i);
      const int64_t route_end_idx = routing.End(i);
      if (depot && depot->depot != -1) {
        _addTimeWindow(time_dimension.CumulVar(route_start_idx),
                       time_windows[depot->depot]);
      }

      if (start_end) {
        auto start_idx = start_end->starts[i];
        if (start_idx != -1)
          _addTimeWindow(time_dimension.CumulVar(route_start_idx),
                         time_windows[start_idx]);

        auto end_idx = start_end->ends[i];
        if (end_idx != -1)
          _addTimeWindow(time_dimension.CumulVar(route_end_idx),
                         time_windows[end_idx]);
      }
    }
  }

  if (_with_vehicle_break_time.has_value()) {
    std::vector<std::vector<TimeWindow>> &break_time =
        _with_vehicle_break_time.value().break_time;

    operations_research::Solver *const solver = routing.solver();
    std::vector<int64_t> node_visit_transit(_duration_matrix.size(), 0);

    if (_with_service_time.has_value()) {
      const auto &service_time = _with_service_time.value().service_time;
      for (int i = 0; i < service_time.size(); ++i) {
        node_visit_transit[i] = service_time[i];
      }
    }

    for (int i = 0; i < break_time.size(); ++i) {
      std::sort(break_time[i].begin(), break_time[i].end());

      std::vector<operations_research::IntervalVar *> break_intervals;
      for (int j = 0; j < break_time[i].size(); ++j) {
        const auto new_var =
            solver
                ->MakeSum(time_dimension.CumulVar(routing.Start(i)),
                          break_time[i][j].start)
                ->Var();
        break_intervals.emplace_back(solver->MakeFixedDurationIntervalVar(
            new_var, break_time[i][j].end - break_time[i][j].start,
            "break time on vehicle " + std::to_string(i) + "on i" +
                std::to_string(j)));
      }

      time_dimension.SetBreakIntervalsOfVehicle(break_intervals, i,
                                                node_visit_transit);
    }
  }

  if (_with_drop_penalties.has_value()) {
    auto &m_penalties_var = _with_drop_penalties.value().penalties;
    if (auto *m_global_penalties = std::get_if<int64_t>(&m_penalties_var);
        m_global_penalties) {
      const auto M = _duration_matrix.size();
      for (int i = 0; i < M; ++i) {
        if (_duration_matrix[i] == std::vector<int64_t>(M, 0)) {
          continue;
        }

        if (depot && depot->depot == i) {
          continue;
        }
        if (start_end) {
          const std::vector<int> &starts = start_end->starts;
          const std::vector<int> &ends = start_end->ends;
          if (std::find(starts.begin(), starts.end(), i) != starts.end()) {
            continue;
          }

          if (std::find(ends.begin(), ends.end(), i) != ends.end()) {
            continue;
          }
        }
        routing.AddDisjunction(
            {manager.NodeToIndex(
                operations_research::RoutingIndexManager::NodeIndex(i))},
            *m_global_penalties);
      }
    }
    if (auto *m_penalties = std::get_if<std::vector<int64_t>>(&m_penalties_var);
        m_penalties) {
      const auto M = _duration_matrix.size();
      for (int i = 0; i < M; ++i) {
        if (_duration_matrix[i] == std::vector<int64_t>(M, 0)) {
          continue;
        }

        if (depot && depot->depot == i) {
          continue;
        }

        if (start_end) {
          const std::vector<int> &starts = start_end->starts;
          const std::vector<int> &ends = start_end->ends;
          if (std::find(starts.begin(), starts.end(), i) != starts.end()) {
            continue;
          }
          if (std::find(ends.begin(), ends.end(), i) != ends.end()) {
            continue;
          }
        }
        routing.AddDisjunction(
            {manager.NodeToIndex(
                operations_research::RoutingIndexManager::NodeIndex(i))},
            m_penalties->at(i));
      }
    }
  }

  for (int i = 0; i < _num_vehicles; ++i) {
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.Start(i)));
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.End(i)));
  }

  int64_t time_limit_sec = 1;
  if (_time_limit.has_value()) {
    time_limit_sec = _time_limit.value();
  }

  // Setting first solution heuristic.
  operations_research::RoutingSearchParameters searchParameters =
      operations_research::DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  searchParameters.set_local_search_metaheuristic(
      operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);

  searchParameters.mutable_time_limit()->set_seconds(time_limit_sec);

  // Solve the problem.
  const operations_research::Assignment *solution =
      routing.SolveWithParameters(searchParameters);
  if (!solution) {
    throw std::runtime_error("No solution found");
  }

  std::vector<RoutingResponse> responses(_num_vehicles);
  for (int vehicle_id = 0; vehicle_id < _num_vehicles; ++vehicle_id) {
    if (!routing.IsVehicleUsed(*solution, vehicle_id)) {
      continue;
    }
    std::vector<int> route;
    int64_t index = routing.Start(vehicle_id);
    while (!routing.IsEnd(index)) {
      auto val = manager.IndexToNode(index).value();
      if (new_index_to_old_index.count(val)) {
        val = new_index_to_old_index.at(val);
      }
      route.push_back(val);
      index = solution->Value(routing.NextVar(index));
    }
    auto val = manager.IndexToNode(index).value();
    if (new_index_to_old_index.count(val)) {
      val = new_index_to_old_index.at(val);
    }
    route.push_back(val);
    auto time_var = time_dimension.CumulVar(index);

    if (depot && depot->depot == -1) {
      route.pop_back();
      route.erase(route.begin());
    }

    if (start_end) {
      if (start_end->starts.at(vehicle_id) == -1) route.erase(route.begin());

      if (start_end->ends.at(vehicle_id) == -1) route.pop_back();
    }

    responses[vehicle_id] = RoutingResponse{
        .route = route,
        .total_duration = solution->Min(time_var),
    };
  }

  return responses;
};

void Routing::_addTimeWindow(operations_research::IntVar *const time_dimension,
                             std::vector<TimeWindow> &time_window) {
  if (!time_dimension) {
    return;
  }
  if (const auto tw_idx = std::find(time_window.begin(), time_window.end(),
                                    TimeWindow{0, INT64_MAX});
      tw_idx != time_window.end()) {
    time_window.erase(tw_idx);
  }

  if (time_window.empty()) {
    return;
  }

  auto earliest_start = time_window[0].start;
  auto latest_end = time_window[time_window.size() - 1].end;

  time_dimension->SetRange(earliest_start, latest_end);
  for (int i = 0; i < time_window.size() - 1; ++i) {
    auto curr_end = time_window[i].end;
    auto next_start = time_window[i + 1].start;

    if (curr_end < next_start) {
      time_dimension->RemoveInterval(curr_end, next_start);
    }
  }
}

void Routing::_addDummyLocAtEnd() {
  auto n = _duration_matrix.size();
  for (int i = 0; i < n; ++i) {
    _duration_matrix[i].emplace_back(0);
  }
  _duration_matrix.emplace_back(std::vector<int64_t>(n + 1, 0));

  if (_with_capacity.has_value()) {
    _with_capacity.value().demands.emplace_back(0);
  }

  if (_with_time_window.has_value()) {
    _with_time_window.value().time_windows.emplace_back(
        std::vector<TimeWindow>{{0, INT64_MAX}});
  }

  if (_with_service_time.has_value()) {
    _with_service_time.value().service_time.push_back(0);
  }

  if (_with_drop_penalties.has_value()) {
    std::vector<int64_t> *penalties = std::get_if<std::vector<int64_t>>(
        &_with_drop_penalties.value().penalties);
    if (penalties) penalties->emplace_back(0);
  }
}

void Routing::_duplicateNodesToBack(int at) {
  size_t n = _duration_matrix.size();
  std::vector<int64_t> duplicated = _duration_matrix[at];
  for (int i = 0; i < n; ++i) {
    _duration_matrix[i].emplace_back(duplicated[i]);
  }
  duplicated.emplace_back(0);
  _duration_matrix.emplace_back(std::move(duplicated));

  if (_with_capacity.has_value()) {
    auto &demands = _with_capacity.value().demands;
    demands.emplace_back(demands[at]);
    auto &capacties = _with_capacity.value().capacities;

    for (auto &cap : capacties) {
      cap += demands[at];
    }
  }

  if (_with_time_window.has_value()) {
    auto &time_windows = _with_time_window.value().time_windows;
    time_windows.emplace_back(time_windows[at]);
  }

  if (_with_service_time.has_value()) {
    auto &service_time = _with_service_time.value().service_time;
    service_time.emplace_back(service_time[at]);
  }

  if (_with_drop_penalties.has_value()) {
    std::vector<int64_t> *penalties = std::get_if<std::vector<int64_t>>(
        &_with_drop_penalties.value().penalties);
    if (penalties) penalties->emplace_back(penalties->at(at));
  }
}

RoutingBuilder Routing::builder() {
  Routing r;
  return RoutingBuilder{r};
}

void RoutingBuilder::_validate() {
  if (_routing._duration_matrix.empty()) {
    throw std::invalid_argument("durationMatrix is empty");
  }

  const auto nodeCount = _routing._duration_matrix.size();
  for (size_t i = 0; i < nodeCount; ++i) {
    if (_routing._duration_matrix[i].size() != nodeCount) {
      throw std::invalid_argument("durationMatrix is not square");
    }
  }

  const auto numVehicle = _routing._num_vehicles;
  if (numVehicle <= 0) {
    throw std::invalid_argument("numVehicles is not positive");
  }

  if (_routing._time_limit.has_value()) {
    const auto time_limit = _routing._time_limit.value();
    if (time_limit <= 0) {
      throw std::invalid_argument("time limit is not positive");
    }
  }

  if (_routing._with_capacity.has_value()) {
    const auto &with_capacity = _routing._with_capacity.value();
    if (with_capacity.capacities.size() != numVehicle) {
      throw std::invalid_argument(
          "capacities size is not equal to numVehicles");
    }
    for (const auto &capacity : with_capacity.capacities) {
      if (capacity <= 0) {
        throw std::invalid_argument("capacities is not positive");
      }
    }

    if (with_capacity.demands.size() != nodeCount) {
      throw std::invalid_argument("demands size is not equal to nodeCount");
    }

    for (const auto &demand : with_capacity.demands) {
      if (demand < 0) {
        throw std::invalid_argument("demands is negative");
      }
    }
  }

  if (_routing._with_pickup_delivery.has_value()) {
    const auto &with_pickup_delivery = _routing._with_pickup_delivery.value();
    const auto &pd = with_pickup_delivery.pickups_deliveries;
    if (pd.empty()) {
      throw std::invalid_argument("pickups_deliveries size is empty");
    }

    for (const auto &pair : pd) {
      if (pair.pickup < 0 || pair.pickup >= nodeCount) {
        throw std::invalid_argument("pickup index is out of range");
      }
      if (pair.delivery < 0 || pair.delivery >= nodeCount) {
        throw std::invalid_argument("delivery index is out of range");
      }

      if (pair.pickup == pair.delivery) {
        throw std::invalid_argument("pickup and delivery index are equal");
      }
    }
  }

  if (_routing._with_time_window.has_value()) {
    const auto &time_windows = _routing._with_time_window.value().time_windows;
    if (time_windows.size() != nodeCount) {
      throw std::invalid_argument(
          "time_windows size is not equal to nodeCount");
    }

    for (const auto &tw : time_windows) {
      if (tw.empty()) {
        throw std::invalid_argument("time_windows is empty");
      }
      for (const auto &t : tw) {
        if (t.start < 0 || t.end < 0) {
          throw std::invalid_argument("time_windows start or end is negative");
        }
        if (t.start > t.end) {
          throw std::invalid_argument("time_windows start is greater than end");
        }
      }
    }
  }

  if (_routing._with_service_time.has_value()) {
    const auto &service_time = _routing._with_service_time.value().service_time;
    if (service_time.size() != nodeCount) {
      throw std::invalid_argument(
          "service_time size is not equal to nodeCount");
    }
    for (const auto &st : service_time) {
      if (st < 0) {
        throw std::invalid_argument("service_time is negative");
      }
    }
  }

  if (_routing._with_drop_penalties.has_value()) {
    const auto penalties_var = _routing._with_drop_penalties.value().penalties;
    if (const auto *const penalty = std::get_if<int64_t>(&penalties_var);
        penalty) {
      if (*penalty < 0) throw std::invalid_argument("penalty is negative");
    } else if (const auto *const penalties =
                   std::get_if<std::vector<int64_t>>(&penalties_var);
               penalties) {
      if (penalties->size() != nodeCount) {
        throw std::invalid_argument("penalties size is not equal to nodeCount");
      }

      for (const auto &penalty : *penalties) {
        if (penalty < 0) {
          throw std::invalid_argument("penalties is negative");
        }
      }
    } else {
      throw std::invalid_argument("penalties is not int64 or vector<int64>");
    }
  }

  if (_routing._with_vehicle_break_time.has_value()) {
    const auto &break_time =
        _routing._with_vehicle_break_time.value().break_time;
    if (break_time.size() != numVehicle) {
      throw std::invalid_argument(
          "break_time size is not equal to numVehicles");
    }

    for (const auto &bt : break_time) {
      if (bt.empty()) {
        throw std::invalid_argument("break_time is empty");
      }

      for (const auto &t : bt) {
        if (t.start < 0 || t.end < 0) {
          throw std::invalid_argument("break_time start or end is negative");
        }
        if (t.start > t.end) {
          throw std::invalid_argument("break_time start is greater than end");
        }
      }
    }
  }
};

Routing RoutingBuilder::build() {
  _validate();
  return _routing;
}

}  // namespace OrtoolsLib