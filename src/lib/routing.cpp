using namespace std;

#include "ortools/constraint_solver/routing.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <variant>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "ortools/base/logging.h"
#include "ortools/constraint_solver/constraint_solver.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"
#include "routing.h"

using namespace operations_research;

namespace ortoolsRouting {
inline void add_dummy_loc_at_end(RoutingConfig &config);
inline void duplicate_nodes_to_back(RoutingConfig &config, int at);
inline void add_time_window(IntVar *const time_dimension,
                            const vector<TimeWindow> &time_window) {
  auto earliest = time_window[0].start;
  auto latest = time_window[time_window.size() - 1].end;
  time_dimension->SetRange(earliest, latest);
  for (int j = 0; j < time_window.size() - 1; j++) {
    auto time_window_end = time_window[j].end;
    auto time_window_start = time_window[j + 1].start;

    time_dimension->RemoveInterval(time_window_end, time_window_start);
  };
}

vector<RoutingResponse> Routing(RoutingConfig config) {
  map<int, int> new_index_to_old_index;
  set<int> pick_drop_set;
  if (config.with_pickup_delivery.has_value()) {
    for (auto &pair : config.with_pickup_delivery.value().pickups_deliveries) {
      if (pick_drop_set.count(pair.pickup)) {
        duplicate_nodes_to_back(config, pair.pickup);
        new_index_to_old_index[config.duration_matrix.size() - 1] = pair.pickup;
        pair.pickup = config.duration_matrix.size() - 1;
      } else {
        pick_drop_set.insert(pair.pickup);
      }

      if (pick_drop_set.count(pair.delivery)) {
        duplicate_nodes_to_back(config, pair.delivery);
        new_index_to_old_index[config.duration_matrix.size() - 1] =
            pair.delivery;
        pair.delivery = config.duration_matrix.size() - 1;
      } else {
        pick_drop_set.insert(pair.delivery);
      }
    }
  }

  optional<RoutingIndexManager> optManager;
  const SingleDepot *depot = get_if<SingleDepot>(&config.depot_config);
  const startEndPair *start_end = get_if<startEndPair>(&config.depot_config);
  if (depot) {
    auto m_depot = depot->depot;
    if (m_depot == -1) {
      add_dummy_loc_at_end(config);
      m_depot = config.duration_matrix.size() - 1;
    }

    if (pick_drop_set.count(m_depot)) {
      duplicate_nodes_to_back(config, m_depot);
      new_index_to_old_index[config.duration_matrix.size() - 1] = m_depot;
      m_depot = config.duration_matrix.size() - 1;
    }

    optManager.emplace(config.duration_matrix.size(), config.num_vehicles,
                       RoutingNodeIndex{m_depot});
  } else if (start_end) {
    vector<RoutingNodeIndex> m_start_nodes(start_end->starts.begin(),
                                           start_end->starts.end());
    vector<RoutingNodeIndex> m_end_nodes(start_end->ends.begin(),
                                         start_end->ends.end());

    if (find(m_start_nodes.begin(), m_start_nodes.end(), -1) !=
            m_start_nodes.end() ||
        find(m_end_nodes.begin(), m_end_nodes.end(), -1) != m_end_nodes.end()) {
      add_dummy_loc_at_end(config);
      for (auto &start : m_start_nodes) {
        if (start == -1) {
          start = config.duration_matrix.size() - 1;
        }
      }
      for (auto &end : m_end_nodes) {
        if (end == -1) {
          end = config.duration_matrix.size() - 1;
        }
      }
    }

    for (auto &start : m_start_nodes) {
      if (pick_drop_set.count(start.value())) {
        duplicate_nodes_to_back(config, start.value());
        new_index_to_old_index[config.duration_matrix.size() - 1] =
            start.value();
        start = config.duration_matrix.size() - 1;
      }
    }

    for (auto &end : m_end_nodes) {
      if (pick_drop_set.count(end.value())) {
        duplicate_nodes_to_back(config, end.value());
        new_index_to_old_index[config.duration_matrix.size() - 1] = end.value();
        end = config.duration_matrix.size() - 1;
      }
    }

    optManager.emplace(config.duration_matrix.size(), config.num_vehicles,
                       m_start_nodes, m_end_nodes);
  } else {
    throw std::invalid_argument("Invalid depot configuration");
  }

  RoutingIndexManager manager = optManager.value();

  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&config, &manager](int64_t from_index, int64_t to_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();

        if (config.with_service_time.has_value()) {
          const int64_t service_time =
              config.with_service_time.value().service_time[from_node];
          return config.duration_matrix[from_node][to_node] + service_time;
        }

        return config.duration_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  const string time = "Time";

  routing.AddDimension(
      transit_callback_index,
      config.with_service_time.has_value()
          ? *max_element(config.with_service_time.value().service_time.begin(),
                         config.with_service_time.value().service_time.end())
          : 0,
      INT64_MAX, !config.with_time_window.has_value(), time);

  RoutingDimension &time_dimension = *routing.GetMutableDimension(time);

  if (config.with_capacity.has_value()) {
    const vector<int64_t> &capacities = config.with_capacity.value().capacities;
    const vector<int64_t> &demands = config.with_capacity.value().demands;
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

  if (config.with_pickup_delivery.has_value()) {
    const vector<PickupDelivery> &pickups_deliveries =
        config.with_pickup_delivery.value().pickups_deliveries;

    Solver *const solver = routing.solver();
    for (const PickupDelivery &pair : pickups_deliveries) {
      const int64_t pickup_index =
          manager.NodeToIndex(RoutingIndexManager::NodeIndex(pair.pickup));
      const int64_t delivery_index =
          manager.NodeToIndex(RoutingIndexManager::NodeIndex(pair.delivery));
      routing.AddPickupAndDelivery(pickup_index, delivery_index);
      solver->AddConstraint(solver->MakeEquality(
          static_cast<IntVar *>(routing.VehicleVar(pickup_index)),
          static_cast<IntVar *>(routing.VehicleVar(delivery_index))));
      solver->AddConstraint(
          solver->MakeLessOrEqual(time_dimension.CumulVar(pickup_index),
                                  time_dimension.CumulVar(delivery_index)));
    }
  }

  if (config.with_time_window.has_value()) {
    vector<vector<TimeWindow>> &time_windows =
        config.with_time_window.value().time_windows;

    for (int i = 0; i < time_windows.size(); ++i) {
      sort(time_windows[i].begin(), time_windows[i].end());

      if (depot && depot->depot == i) {
        continue;
      }

      if (start_end) {
        const vector<int> &starts = start_end->starts;
        const vector<int> &ends = start_end->ends;
        if (find(starts.begin(), starts.end(), i) != starts.end()) {
          continue;
        }
        if (find(ends.begin(), ends.end(), i) != ends.end()) {
          continue;
        }
      }

      add_time_window(time_dimension.CumulVar(manager.NodeToIndex(
                          RoutingIndexManager::NodeIndex(i))),
                      time_windows[i]);
    }

    for (int i = 0; i < config.num_vehicles; ++i) {
      const int64_t route_start_idx = routing.Start(i);
      const int64_t route_end_idx = routing.End(i);
      if (depot && depot->depot != -1) {
        add_time_window(time_dimension.CumulVar(route_start_idx),
                        time_windows[depot->depot]);
      }

      if (start_end) {
        auto start_idx = start_end->starts[i];
        if (start_idx != -1)
          add_time_window(time_dimension.CumulVar(route_start_idx),
                          time_windows[start_idx]);

        auto end_idx = start_end->ends[i];
        if (end_idx != -1)
          add_time_window(time_dimension.CumulVar(route_end_idx),
                          time_windows[end_idx]);
      }
    }
  }

  if (config.with_vehicle_break_time.has_value()) {
    const vector<vector<TimeWindow>> &break_time =
        config.with_vehicle_break_time.value().break_time;

    Solver *const solver = routing.solver();
    vector<int64_t> node_visit_transit(config.duration_matrix.size(), 0);

    if (config.with_service_time.has_value()) {
      const auto &service_time = config.with_service_time.value().service_time;
      for (int i = 0; i < service_time.size(); ++i) {
        node_visit_transit[i] = service_time[i];
      }
    }

    for (int i = 0; i < break_time.size(); ++i) {
      sort(break_time[i].begin(), break_time[i].end());

      vector<IntervalVar *> break_intervals;
      for (int j = 0; j < break_time[i].size(); ++j) {
        break_intervals.emplace_back(solver->MakeFixedInterval(
            break_time[i][j].start,
            break_time[i][j].end - break_time[i][j].start,
            "break time on vehicle " + to_string(i) + "on i" + to_string(j)));
      }

      time_dimension.SetBreakIntervalsOfVehicle(break_intervals, i,
                                                node_visit_transit);
    }
  }

  if (config.with_drop_penalties.has_value()) {
    auto &m_penalties_var = config.with_drop_penalties.value().penalties;
    if (auto *m_global_penalties = get_if<int64_t>(&m_penalties_var);
        m_global_penalties) {
      for (int i = 0; i < config.duration_matrix.size(); ++i) {
        if (depot && depot->depot == i) {
          continue;
        }
        if (start_end) {
          const vector<int> &starts = start_end->starts;
          const vector<int> &ends = start_end->ends;
          if (find(starts.begin(), starts.end(), i) != starts.end()) {
            continue;
          }
          if (find(ends.begin(), ends.end(), i) != ends.end()) {
            continue;
          }
        }
        routing.AddDisjunction(
            {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))},
            *m_global_penalties);
      }
    }
    if (auto *m_penalties = get_if<vector<int64_t>>(&m_penalties_var);
        m_penalties) {
      for (int i = 0; i < config.duration_matrix.size(); ++i) {
        if (depot && depot->depot == i) {
          continue;
        }
        if (start_end) {
          const vector<int> starts = start_end->starts;
          const vector<int> ends = start_end->ends;
          if (find(starts.begin(), starts.end(), i) != starts.end()) {
            continue;
          }
          if (find(ends.begin(), ends.end(), i) != ends.end()) {
            continue;
          }
        }
        routing.AddDisjunction(
            {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))},
            m_penalties->at(i));
      }
    }
  }

  for (int i = 0; i < config.num_vehicles; ++i) {
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.Start(i)));
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.End(i)));
  }

  int64_t time_limit_sec = 1;
  if (config.time_limit.has_value()) {
    time_limit_sec = config.time_limit.value();
  }

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  searchParameters.set_local_search_metaheuristic(
        LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  
  searchParameters.mutable_time_limit()->set_seconds(time_limit_sec);

  // Solve the problem.
  const Assignment *solution = routing.SolveWithParameters(searchParameters);
  if (!solution) {
    throw std::runtime_error("No solution found");
  }

  vector<RoutingResponse> responses(config.num_vehicles);
  for (int vehicle_id = 0; vehicle_id < config.num_vehicles; ++vehicle_id) {
    if (!routing.IsVehicleUsed(*solution, vehicle_id)) {
      continue;
    }
    vector<int> route;
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
}

inline void add_dummy_loc_at_end(RoutingConfig &config) {
  auto n = config.duration_matrix.size();
  for (int i = 0; i < n; ++i) {
    config.duration_matrix[i].emplace_back(0);
  }
  config.duration_matrix.emplace_back(vector<int64_t>(n + 1, 0));

  if (config.with_capacity.has_value()) {
    config.with_capacity.value().demands.emplace_back(0);
  }

  if (config.with_time_window.has_value()) {
    config.with_time_window.value().time_windows.emplace_back(0, INT64_MAX);
  }

  if (config.with_service_time.has_value()) {
    config.with_service_time.value().service_time.push_back(0);
  }

  if (config.with_drop_penalties.has_value()) {
    vector<int64_t> *penalties =
        get_if<vector<int64_t>>(&config.with_drop_penalties.value().penalties);
    if (penalties) penalties->emplace_back(0);
  }
}

inline void duplicate_nodes_to_back(RoutingConfig &config, int at) {
  size_t n = config.duration_matrix.size();
  vector<int64_t> duplicated = config.duration_matrix[at];
  for (int i = 0; i < n; ++i) {
    config.duration_matrix[i].emplace_back(duplicated[i]);
  }
  duplicated.emplace_back(0);
  config.duration_matrix.emplace_back(duplicated);

  if (config.with_capacity.has_value()) {
    auto &demands = config.with_capacity.value().demands;
    demands.emplace_back(demands[at]);
    auto &capacties = config.with_capacity.value().capacities;

    for (auto &cap : capacties) {
      cap += demands[at];
    }
  }

  if (config.with_time_window.has_value()) {
    auto &time_windows = config.with_time_window.value().time_windows;
    time_windows.emplace_back(time_windows[at]);
  }

  if (config.with_service_time.has_value()) {
    auto &service_time = config.with_service_time.value().service_time;
    service_time.emplace_back(service_time[at]);
  }

  if (config.with_drop_penalties.has_value()) {
    vector<int64_t> *penalties =
        get_if<vector<int64_t>>(&config.with_drop_penalties.value().penalties);
    if (penalties) penalties->emplace_back(penalties->at(at));
  }
}
}  // namespace ortoolsRouting