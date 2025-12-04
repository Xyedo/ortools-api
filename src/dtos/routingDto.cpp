
#include "routingDto.h"

#include <cstdint>
#include <format>
#include <json/json.h>
#include <lib/routing.h>
#include <optional>
#include <routing-proto/routing.grpc.pb.h>
#include <variant>
#include <vector>

namespace RoutingDTO {
RoutingModel intoEntity(const routing::RoutingRequest *const request) noexcept {
  std::vector<std::vector<int64_t>> duration_matrix;
  for (const auto &row : request->durationmatrix()) {
    std::vector<int64_t> row_vector(row.value().begin(), row.value().end());
    duration_matrix.push_back(row_vector);
  }

  std::variant<OrtoolsLib::SingleDepot, OrtoolsLib::startEndPair> depot_config;
  if (request->has_depot()) {
    depot_config.emplace<OrtoolsLib::SingleDepot>(
        OrtoolsLib::SingleDepot{.depot = request->depot()});
  }
  if (request->has_startend()) {
    depot_config.emplace<OrtoolsLib::startEndPair>(OrtoolsLib::startEndPair{
        .starts = std::vector<int32_t>(request->startend().start().begin(),
                                       request->startend().start().end()),
        .ends = std::vector<int32_t>(request->startend().end().begin(),
                                     request->startend().end().end()),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithCapacity> with_capacity;
  if (request->has_withcapacity()) {
    with_capacity.emplace(OrtoolsLib::RoutingOptionWithCapacity{
        .capacities = std::vector<int64_t>(
            request->withcapacity().vehiclecapacity().begin(),
            request->withcapacity().vehiclecapacity().end()),
        .demands =
            std::vector<int64_t>(request->withcapacity().demands().begin(),
                                 request->withcapacity().demands().end()),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithPickupDelivery>
      with_pickup_delivery;
  if (request->has_withpickupanddeliveries()) {
    std::vector<OrtoolsLib::PickupDelivery> pickups_deliveries;
    for (const auto &pickup_delivery :
         request->withpickupanddeliveries().pickupdrops()) {
      pickups_deliveries.emplace_back(OrtoolsLib::PickupDelivery{
          .pickup = pickup_delivery.a(),
          .delivery = pickup_delivery.b(),
      });
    }
    with_pickup_delivery.emplace(OrtoolsLib::RoutingOptionWithPickupDelivery{
        .pickups_deliveries = std::move(pickups_deliveries),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithTimeWindow> with_time_window;
  if (request->has_withtimewindows()) {
    std::vector<std::vector<OrtoolsLib::TimeWindow>> time_windows;
    for (const auto &tws : request->withtimewindows().timewindows()) {
      std::vector<OrtoolsLib::TimeWindow> time_window;
      for (const auto &tw : tws.pairs()) {
        time_window.emplace_back(OrtoolsLib::TimeWindow{
            .start = tw.a(),
            .end = tw.b(),
        });
      }

      time_windows.emplace_back(time_window);
    }

    with_time_window.emplace(OrtoolsLib::RoutingOptionWithTimeWindow{
        .time_windows = std::move(time_windows),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithServiceTime> with_service_time;
  if (request->has_withservicetime()) {
    with_service_time.emplace(OrtoolsLib::RoutingOptionWithServiceTime{
        .service_time = std::vector<int64_t>(
            request->withservicetime().servicetime().begin(),
            request->withservicetime().servicetime().end()),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithPenalties> with_drop_penalties;
  if (request->has_withpenalties()) {
    if (request->withpenalties().has_penalty()) {
      with_drop_penalties.emplace(OrtoolsLib::RoutingOptionWithPenalties{
          .penalties = request->withpenalties().penalty(),
      });
    } else if (request->withpenalties().has_penalties()) {
      std::vector<int64_t> penalties(
          request->withpenalties().penalties().value().begin(),
          request->withpenalties().penalties().value().end());

      with_drop_penalties.emplace(OrtoolsLib::RoutingOptionWithPenalties{
          .penalties = std::move(penalties),
      });
    }
  }

  std::optional<OrtoolsLib::RoutingOptionWithVehicleBreakTime>
      with_vehicle_break_time;
  if (request->has_withbreaktime()) {
    std::vector<std::vector<OrtoolsLib::TimeWindow>> break_time;
    for (const auto &breaks : request->withbreaktime().breaktimes()) {
      std::vector<OrtoolsLib::TimeWindow> time_windows;
      for (const auto &tw : breaks.pairs()) {
        time_windows.emplace_back(OrtoolsLib::TimeWindow{
            .start = tw.a(),
            .end = tw.b(),
        });
      }

      break_time.emplace_back(time_windows);
    }

    with_vehicle_break_time.emplace(
        OrtoolsLib::RoutingOptionWithVehicleBreakTime{
            .break_time = std::move(break_time),
        });
  }

  return RoutingModel{
      .duration_matrix = std::move(duration_matrix),
      .depot_config = std::move(depot_config),
      .num_vehicles = request->numvehicles(),
      .time_limit = request->apitimelimit(),
      .with_capacity = std::move(with_capacity),
      .with_pickup_delivery = std::move(with_pickup_delivery),
      .with_time_window = std::move(with_time_window),
      .with_service_time = std::move(with_service_time),
      .with_drop_penalties = std::move(with_drop_penalties),
      .with_vehicle_break_time = std::move(with_vehicle_break_time),
  };
}

RoutingModel parseJSON(std::shared_ptr<Json::Value> json) {
  if (!json) {
    throw ParseErrorElement("json is null");
  }

  if (!(*json).isMember("durationMatrix") ||
      !(*json)["durationMatrix"].isArray()) {
    throw ParseErrorElement("durationMatrix", {"expected arrays"});
  }

  std::vector<std::vector<int64_t>> duration_matrix;
  duration_matrix.reserve((*json)["durationMatrix"].size());

  for (int i = 0; i < (*json)["durationMatrix"].size(); ++i) {
    const auto &row = (*json)["durationMatrix"][i];
    if (!row.isArray()) {
      throw ParseErrorElement(std::format("durationMatrix[{}]", i),
                              {"expected arrays"});
    }

    std::vector<int64_t> row_vector;
    row_vector.reserve(row.size());
    for (const auto &value : row) {
      if (!value.isInt64()) {
        throw ParseErrorElement(std::format("durationMatrix[{}]", i),
                                {"value is not integer"});
      }

      row_vector.push_back(value.asInt64());
    }

    duration_matrix.push_back(row_vector);
  }
  int32_t num_vehicles = 1;
  if ((*json).isMember("numVehicles")) {
    if (!(*json)["numVehicles"].isInt()) {
      throw ParseErrorElement("numVehicles", {"value is not integer"});
    }

    num_vehicles = (*json)["numVehicles"].asInt();
  }

  std::variant<OrtoolsLib::SingleDepot, OrtoolsLib::startEndPair> depot_config;
  if (!(*json).isMember("routingMode")) {
    throw ParseErrorElement("routingMode", {"value is required"});
  }

  if (!(*json)["routingMode"].isMember("type")) {
    throw ParseErrorElement("routingMode.type", {"value is required"});
  }
  if (!(*json)["routingMode"].isMember("payload")) {
    // throw std::invalid_argument("routingMode.payload is required");
    throw ParseErrorElement("routingMode.payload", {"value is required"});
  }

  if (!(*json)["routingMode"]["type"].isString()) {
    // throw std::invalid_argument("routingMode.type is expected to be string");
    throw ParseErrorElement("routingMode.type",
                            {"value is expected to be string"});
  }

  if (auto routingModeType = (*json)["routingMode"]["type"].asString();
      routingModeType == "depot") {
    if (!(*json)["routingMode"]["payload"].isMember("depot")) {
      // throw std::invalid_argument("routingMode.payload.depot is required");
      throw ParseErrorElement("routingMode.payload.depot",
                              {"value is required"});
    }
    if (!(*json)["routingMode"]["payload"]["depot"].isInt()) {
      // throw std::invalid_argument(
      //     "routingMode.payload.depot is expected to be int");
      throw ParseErrorElement("routingMode.payload.depot",
                              {"value is expected to be int"});
    }

    depot_config.emplace<OrtoolsLib::SingleDepot>(OrtoolsLib::SingleDepot{
        .depot = (*json)["routingMode"]["payload"]["depot"].asInt()});

  } else if (routingModeType == "startEnd") {
    if (!(*json)["routingMode"]["payload"].isMember("starts")) {
      // throw std::invalid_argument("routingMode.payload.starts is required");
      throw ParseErrorElement("routingMode.payload.starts",
                              {"value is required"});
    }
    if (!(*json)["routingMode"]["payload"].isMember("ends")) {
      // throw std::invalid_argument("routingMode.payload.ends is required");
      throw ParseErrorElement("routingMode.payload.ends",
                              {"value is required"});
    }

    if (!(*json)["routingMode"]["payload"]["starts"].isArray()) {
      // throw std::invalid_argument(
      //     "routingMode.payload.starts is expected to be an array");
      throw ParseErrorElement("routingMode.payload.starts",
                              {"expected to be an array"});
    }
    if (!(*json)["routingMode"]["payload"]["ends"].isArray()) {
      throw ParseErrorElement("routingMode.payload.ends",
                              {"expected to be an array"});
    }
    std::vector<int> starts;
    starts.reserve((*json)["routingMode"]["payload"]["starts"].size());

    for (int i = 0; i < (*json)["routingMode"]["payload"]["starts"].size();
         i++) {
      const auto &value = (*json)["routingMode"]["payload"]["starts"][i];
      if (!value.isInt()) {
        throw ParseErrorElement(
            std::format("routingMode.payload.starts[{}]", i),
            {"expected to be an integer"});
      }

      starts.push_back(value.asInt64());
    }

    std::vector<int> ends;
    ends.reserve((*json)["routingMode"]["payload"]["ends"].size());

    for (int i = 0; i < (*json)["routingMode"]["payload"]["ends"].size(); ++i) {
      const auto &value = (*json)["routingMode"]["payload"]["ends"][i];
      if (!value.isInt()) {
        throw ParseErrorElement(std::format("routingMode.payload.ends[{}]", i),
                                {"expected to be an integer"});
      }

      ends.push_back(value.asInt64());
    }
    depot_config.emplace<OrtoolsLib::startEndPair>(OrtoolsLib::startEndPair{
        .starts = std::move(starts),
        .ends = std::move(ends),
    });
  } else {
    // throw std::invalid_argument(
    //     "routingMode.type is expected to be enum of 'depot' | 'startEnd'");
    throw ParseErrorElement("routingMode.type",
                            {"expected to be enum of 'depot' | 'startEnd'"});
  }

  int64_t apiTimeLimit = 1;
  if ((*json).isMember("apiTimeLimit")) {
    if (!(*json)["apiTimeLimit"].isInt64()) {
      apiTimeLimit = (*json)["apiTimeLimit"].asInt64();
    }
  }

  std::optional<OrtoolsLib::RoutingOptionWithCapacity> with_capacity;
  if ((*json).isMember("withCapacity")) {
    if (!(*json)["withCapacity"].isMember("vehicleCapacity")) {
      // throw std::invalid_argument("withCapacity.vehicleCapacity is
      // required");
      throw ParseErrorElement("withCapacity.vehicleCapacity",
                              {"value is required"});
    }
    if (!(*json)["withCapacity"].isMember("demands")) {
      // throw std::invalid_argument("withCapacity.demands is required");
      throw ParseErrorElement("withCapacity.demands", {"value is required"});
    }

    if (!(*json)["withCapacity"]["vehicleCapacity"].isArray()) {
      // throw std::invalid_argument(
      // "withCapacity.vehicleCapacity is expected to be an array");
      throw ParseErrorElement("withCapacity.vehicleCapacity",
                              {"value is expected to be an array"});
    }
    if (!(*json)["withCapacity"]["demands"].isArray()) {
      // throw std::invalid_argument(
      //     "withCapacity.demands is expected to be an array");
      throw ParseErrorElement("withCapacity.demands",
                              {"value is expected to be an array"});
    }

    std::vector<int64_t> vehicle_capacity;
    vehicle_capacity.reserve((*json)["withCapacity"]["vehicleCapacity"].size());

    for (const auto &value : (*json)["withCapacity"]["vehicleCapacity"]) {
      if (!value.isInt64()) {
        throw ParseErrorElement("withCapacity.vehicleCapacity",
                                {"value is not integer"});
      }

      vehicle_capacity.push_back(value.asInt64());
    }

    std::vector<int64_t> demands;
    demands.reserve((*json)["withCapacity"]["demands"].size());

    for (const auto &value : (*json)["withCapacity"]["demands"]) {
      if (!value.isInt64()) {
        // throw std::invalid_argument("withCapacity.demands value is not
        // int64");
        throw ParseErrorElement("withCapacity.demands",
                                {"value is not integer"});
      }

      demands.push_back(value.asInt64());
    }
    with_capacity.emplace(OrtoolsLib::RoutingOptionWithCapacity{
        .capacities = std::move(vehicle_capacity),
        .demands = std::move(demands),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithPickupDelivery>
      with_pickup_delivery;
  if ((*json).isMember("withPickupAndDeliveries")) {
    if (!(*json)["withPickupAndDeliveries"].isMember("pickDrops")) {
      // throw std::invalid_argument(
      //     "withPickupAndDeliveries.pickDrops is required");
      throw ParseErrorElement("withPickupAndDeliveries.pickDrops",
                              {"value is required"});
    }

    if (!(*json)["withPickupAndDeliveries"]["pickDrops"].isArray()) {
      // throw std::invalid_argument(
      //     "withPickupDelivery.pickupsDeliveries is expected to be an array");
      throw ParseErrorElement("withPickupAndDeliveries.pickDrops",
                              {"value is expected to be an array"});
    }

    std::vector<OrtoolsLib::PickupDelivery> pickups_deliveries;
    pickups_deliveries.reserve(
        (*json)["withPickupAndDeliveries"]["pickDrops"].size());

    for (const auto &value : (*json)["withPickupAndDeliveries"]["pickDrops"]) {
      if (!value.isMember("pickup")) {
        throw ParseErrorElement("withPickupAndDeliveries.pickDrops.pickup",
                                {"value is required"});
      }

      if (!value.isMember("drop")) {
        throw ParseErrorElement("withPickupAndDeliveries.pickDrops.drop",
                                {"value is required"});
      }

      pickups_deliveries.emplace_back(OrtoolsLib::PickupDelivery{
          .pickup = value["pickup"].asInt64(),
          .delivery = value["drop"].asInt64(),
      });
    }
    with_pickup_delivery.emplace(OrtoolsLib::RoutingOptionWithPickupDelivery{
        .pickups_deliveries = std::move(pickups_deliveries),
    });
  }
  std::optional<OrtoolsLib::RoutingOptionWithTimeWindow> with_time_window;
  if ((*json).isMember("withTimeWindows")) {
    if (!(*json)["withTimeWindows"].isMember("timeWindows")) {
      throw ParseErrorElement("withTimeWindows.timeWindows",
                              {"value is required"});
    }

    if (!(*json)["withTimeWindows"]["timeWindows"].isArray()) {
      throw ParseErrorElement("withTimeWindows.timeWindows",
                              {"value is expected to be an array"});
    }

    std::vector<std::vector<OrtoolsLib::TimeWindow>> time_windows;
    time_windows.reserve((*json)["withTimeWindows"]["timeWindows"].size());

    for (int i = 0; i < (*json)["withTimeWindows"]["timeWindows"].size(); ++i) {
      const auto &value = (*json)["withTimeWindows"]["timeWindows"][i];
      if (!value.isArray()) {
        throw ParseErrorElement(
            std::format("withTimeWindow.timeWindows[{}]", i),
            {"value is not an array"});
      }

      std::vector<OrtoolsLib::TimeWindow> time_window;
      time_window.reserve(value.size());

      for (int j = 0; j < value.size(); ++j) {
        const auto &tw = value[j];
        if (!tw.isMember("start")) {
          throw ParseErrorElement(
              std::format("withTimeWindow.timeWindows[{}][{}].start", i, j),
              {"value is required"});
        }

        if (!tw.isMember("end")) {
          throw ParseErrorElement(
              std::format("withTimeWindow.timeWindows[{}][{}].end", i, j),
              {"value is required"});
        }

        time_window.emplace_back(OrtoolsLib::TimeWindow{
            .start = tw["start"].asInt64(),
            .end = tw["end"].asInt64(),
        });
      }
      time_windows.emplace_back(std::move(time_window));
    }
    with_time_window.emplace(OrtoolsLib::RoutingOptionWithTimeWindow{
        .time_windows = std::move(time_windows),
    });
  }

  std::optional<OrtoolsLib::RoutingOptionWithServiceTime> with_service_time;
  if ((*json).isMember("withServiceTime")) {
    if (!(*json)["withServiceTime"].isMember("serviceTime")) {
      // throw std::invalid_argument("withServiceTime.serviceTime is required");
      throw ParseErrorElement("withServiceTime.serviceTime",
                              {"value is required"});
    }

    if (!(*json)["withServiceTime"]["serviceTime"].isArray()) {
      // throw std::invalid_argument(
      //     "withServiceTime.serviceTime is expected to be an array");
      throw ParseErrorElement("withServiceTime.serviceTime",
                              {"value is expected to be an array"});
    }

    std::vector<int64_t> service_time;
    service_time.reserve((*json)["withServiceTime"]["serviceTime"].size());

    for (int i = 0; i < (*json)["withServiceTime"]["serviceTime"].size(); ++i) {
      const auto &value = (*json)["withServiceTime"]["serviceTime"][i];
      if (!value.isInt64()) {
        // throw std::invalid_argument(
        //     "withServiceTime.serviceTime value is not int64");
        throw ParseErrorElement(
            std::format("withServiceTime.serviceTime[{}]", i),
            {"value is expected to be int64"});
      }

      service_time.push_back(value.asInt64());
    }
    with_service_time.emplace(OrtoolsLib::RoutingOptionWithServiceTime{
        .service_time = std::move(service_time),
    });
  }
  std::optional<OrtoolsLib::RoutingOptionWithPenalties> with_drop_penalties;
  if ((*json).isMember("withDropPenalties")) {
    if ((*json)["withDropPenalties"].isMember("penalty")) {
      if (!(*json)["withDropPenalties"]["penalty"].isInt64()) {
        throw ParseErrorElement("withDropPenalties.penalty",
                                {"value is expected to be int64"});
      }
      with_drop_penalties.emplace(OrtoolsLib::RoutingOptionWithPenalties{
          .penalties = (*json)["withDropPenalties"]["penalty"].asInt64(),
      });
    } else if ((*json)["withDropPenalties"].isMember("penalties")) {
      if (!(*json)["withDropPenalties"]["penalties"].isArray()) {
        throw ParseErrorElement("withDropPenalties.penalties",
                                {"value is expected to be an array"});
      }

      std::vector<int64_t> penalties;
      penalties.reserve((*json)["withDropPenalties"]["penalties"].size());

      for (int i = 0; i < (*json)["withDropPenalties"]["penalties"].size();
           ++i) {
        const auto &value = (*json)["withDropPenalties"]["penalties"][i];
        if (!value.isInt64()) {
          throw ParseErrorElement(
              std::format("withDropPenalties.penalties[{}]", i),
              {"value is expected to be int64"});
        }

        penalties.push_back(value.asInt64());
      }
      with_drop_penalties.emplace(OrtoolsLib::RoutingOptionWithPenalties{
          .penalties = std::move(penalties),
      });
    }
  }

  std::optional<OrtoolsLib::RoutingOptionWithVehicleBreakTime>
      with_vehicle_break_time;
  if ((*json).isMember("withVehicleBreakTime")) {
    if (!(*json)["withVehicleBreakTime"].isMember("breakTimes")) {
      throw ParseErrorElement("withVehicleBreakTime.breakTimes",
                              {"breakTimes is required"});
    }

    if (!(*json)["withVehicleBreakTime"]["breakTimes"].isArray()) {
      throw ParseErrorElement("withVehicleBreakTime.breakTimes",
                              {"breakTimes is expected to be an array"});
    }

    std::vector<std::vector<OrtoolsLib::TimeWindow>> break_time;
    break_time.reserve((*json)["withVehicleBreakTime"]["breakTimes"].size());

    for (int i = 0; i < (*json)["withVehicleBreakTime"]["breakTimes"].size();
         ++i) {
      const auto &value = (*json)["withVehicleBreakTime"]["breakTimes"][i];
      if (!value.isArray()) {
        throw ParseErrorElement(
            std::format("withVehicleBreakTime.breakTimes[{}]", i),
            {"value isexpected to be an array"});
      }

      std::vector<OrtoolsLib::TimeWindow> time_windows;
      time_windows.reserve(value.size());

      for (int j = 0; j < value.size(); ++j) {
        const auto &tw = value[j];
        if (!tw.isMember("start")) {
          throw ParseErrorElement(
              std::format("withVehicleBreakTime.breakTimes[{}][{}].start", i,
                          j),
              {"value is required"});
        }
        if (!tw.isMember("end")) {
          throw ParseErrorElement(
              std::format("withVehicleBreakTime.breakTimes[{}][{}].end", i, j),
              {"value is required"});
        }

        time_windows.emplace_back(OrtoolsLib::TimeWindow{
            .start = tw["start"].asInt64(),
            .end = tw["end"].asInt64(),
        });
      }
      break_time.emplace_back(std::move(time_windows));
    }
    with_vehicle_break_time.emplace(
        OrtoolsLib::RoutingOptionWithVehicleBreakTime{
            .break_time = std::move(break_time),
        });
  }

  return RoutingModel{
      .duration_matrix = std::move(duration_matrix),
      .depot_config = std::move(depot_config),
      .num_vehicles = num_vehicles,
      .time_limit = apiTimeLimit,
      .with_capacity = std::move(with_capacity),
      .with_pickup_delivery = std::move(with_pickup_delivery),
      .with_time_window = std::move(with_time_window),
      .with_service_time = std::move(with_service_time),
      .with_drop_penalties = std::move(with_drop_penalties),
      .with_vehicle_break_time = std::move(with_vehicle_break_time),
  };
}
} // namespace RoutingDTO
