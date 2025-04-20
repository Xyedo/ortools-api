#include "routingDto.h"

#include <gtest/gtest.h>
#include <routing-proto/routing.grpc.pb.h>

#include <vector>

#include "lib/routing.h"

TEST(RoutingDTO, TestIntoRoutingModel) {
  routing::RoutingRequest request;
  request.set_numvehicles(1);
  std::vector<std::vector<int64_t>> duration_matrix{
      {0, 1, 2, 3},
      {1, 0, 4, 5},
      {2, 4, 0, 6},
      {3, 5, 6, 0},
  };

  for (const auto &row : duration_matrix) {
    auto *row_proto = request.add_durationmatrix();
    for (const auto &value : row) {
      row_proto->add_value(value);
    }
  }

  OrtoolsLib::startEndPair depot_config{.starts{0}, .ends{-1}};
  auto *start_end = new routing::startEndVehicle();
  start_end->add_start(0);
  start_end->add_end(-1);
  request.set_allocated_startend(start_end);

  OrtoolsLib::RoutingOptionWithServiceTime service_time_config{
      .service_time{0, 1, 1, 1}};

  auto *service_time = new routing::RoutingRequestWithServiceTime();
  for (const auto &value : service_time_config.service_time) {
    service_time->add_servicetime(value);
  }
  request.set_allocated_withservicetime(service_time);

  std::vector<int64_t> demands{5, 10, 10, 30};

  OrtoolsLib::RoutingOptionWithCapacity cap{
      .capacities = {100},
      .demands = demands,
  };
  auto *capacity_proto = new routing::RoutingRequestWithCapacity();
  for (const auto &demand : demands) {
    capacity_proto->add_demands(demand);
  }
  capacity_proto->add_vehiclecapacity(100);
  request.set_allocated_withcapacity(capacity_proto);

  std::vector<OrtoolsLib::PickupDelivery> pickups_deliveries = {
      {2, 0},
      {3, 1},
      {3, 2},
  };

  OrtoolsLib::RoutingOptionWithPickupDelivery pd{
      .pickups_deliveries = pickups_deliveries,
  };
  auto *pickup_delivery = new routing::RoutingRequestWithPickupAndDeliveries();
  for (const auto &pd : pickups_deliveries) {
    auto *pair = pickup_delivery->add_pickupdrops();
    pair->set_a(pd.pickup);
    pair->set_b(pd.delivery);
  }

  request.set_allocated_withpickupanddeliveries(pickup_delivery);

  OrtoolsLib::RoutingOptionWithPenalties dropPinalties{.penalties = 1000};

  auto *penalties = new routing::RoutingRequestWithPenalties();
  penalties->set_penalty(1000);
  request.set_allocated_withpenalties(penalties);

  std::vector<std::vector<OrtoolsLib::TimeWindow>> time_windows{
      {{0, 40}},
      {{10, 50}},
      {{20, 60}},
      {{30, 70}},
  };

  auto *time_window = new routing::RoutingRequestWithTimeWindows();
  for (const auto &tw : time_windows) {
    auto *time_window_proto = time_window->add_timewindows();
    for (const auto &window : tw) {
      auto *pair = time_window_proto->add_pairs();
      pair->set_a(window.start);
      pair->set_b(window.end);
    }
  }
  request.set_allocated_withtimewindows(time_window);
  OrtoolsLib::RoutingOptionWithTimeWindow tw{
      .time_windows = time_windows,
  };

  std::vector<std::vector<OrtoolsLib::TimeWindow>> break_time{
      {{2, 3}},
  };
  auto *break_time_proto = new routing::RoutingRequestWIthVehicleBreakTime();
  for (const auto &bt : break_time) {
    auto *bt_proto = break_time_proto->add_breaktimes();
    for (const auto &window : bt) {
      auto *pair = bt_proto->add_pairs();
      pair->set_a(window.start);
      pair->set_b(window.end);
    }
  }

  request.set_allocated_withbreaktime(break_time_proto);
  OrtoolsLib::RoutingOptionWithVehicleBreakTime bt{
      .break_time = break_time,
  };

  RoutingDTO::RoutingModel expected{
      .duration_matrix = duration_matrix,
      .depot_config = depot_config,
      .with_capacity = cap,
      .with_pickup_delivery = pd,
      .with_time_window = tw,
      .with_service_time = service_time_config,
      .with_drop_penalties = dropPinalties,
      .with_vehicle_break_time = bt,
  };
  auto result = RoutingDTO::intoEntity(&request);
  EXPECT_EQ(result.duration_matrix, expected.duration_matrix);
  EXPECT_EQ(std::get<OrtoolsLib::startEndPair>(result.depot_config).starts,
            std::get<OrtoolsLib::startEndPair>(expected.depot_config).starts);
  EXPECT_EQ(result.num_vehicles, expected.num_vehicles);
  EXPECT_EQ(result.time_limit, expected.time_limit);
  EXPECT_EQ(result.with_capacity.value().capacities,
            expected.with_capacity.value().capacities);
  EXPECT_EQ(result.with_capacity.value().demands,
            expected.with_capacity.value().demands);
  EXPECT_EQ(result.with_pickup_delivery.value().pickups_deliveries,
            expected.with_pickup_delivery.value().pickups_deliveries);
  EXPECT_EQ(result.with_time_window.value().time_windows,
            expected.with_time_window.value().time_windows);
  EXPECT_EQ(result.with_service_time.value().service_time,
            expected.with_service_time.value().service_time);
  EXPECT_EQ(result.with_drop_penalties.value().penalties,
            expected.with_drop_penalties.value().penalties);
  EXPECT_EQ(result.with_vehicle_break_time.value().break_time,
            expected.with_vehicle_break_time.value().break_time);
}

TEST(RoutingDTO, TestParsingJSON) {
  const std::string rawJson{R"(
      {
        "durationMatrix": [
          [
            1,
            2,
            3
          ],
          [
            1,
            2,
            3
          ],
          [
            1,
            2,
            3
          ]
        ],
        "numVehicles": 2,
        "routingMode": {
          "type": "startEnd",
          "payload": {
            "starts": [
              1,
              2
            ],
            "ends": [
              1,
              2
            ]
          }
        },
        "apiTimeLimit": 1,
        "withCapacity": {
          "vehicleCapacity": [
            1,
            2
          ],
          "demands": [
            1,
            2,
            3
          ]
        },
        "withPickupAndDeliveries": {
          "pickDrops": [
            {
              "pickup": 1,
              "drop": 2
            }
          ]
        },
        "withTimeWindows": {
          "timeWindows": [
            [
              {
                "start": 1,
                "end": 2
              }
            ]
          ]
        },
        "withServiceTime": {
          "serviceTime": [
            1,
            1,
            1
          ]
        },
        "withDropPenalties": {
          "penalty": 1
        },
        "withVehicleBreakTime": {
          "breakTimes": [
            [
              {
                "start": 1,
                "end": 2
              }
            ]
          ]
        }
      }
    )"};

  const auto rawJsonLength = static_cast<int>(rawJson.length());

  Json::CharReaderBuilder builder;
  const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value root;
  Json::String err;
  ASSERT_TRUE(reader->parse(rawJson.c_str(), rawJson.c_str() + rawJsonLength,
                            &root, &err));
  ASSERT_TRUE(err.empty());
  auto routing_model =
      RoutingDTO::parseJSON(std::make_shared<Json::Value>(root));
  std::vector<std::vector<int64_t>> expected_duration_matrix{
      {1, 2, 3},
      {1, 2, 3},
      {1, 2, 3},
  };
  ASSERT_EQ(routing_model.duration_matrix, expected_duration_matrix);
  ASSERT_EQ(routing_model.num_vehicles, 2);
  ASSERT_EQ(routing_model.time_limit, 1);
  OrtoolsLib::startEndPair expected_depot_config{
      .starts = {1, 2},
      .ends = {1, 2},
  };
  ASSERT_TRUE(std::holds_alternative<OrtoolsLib::startEndPair>(
      routing_model.depot_config));
  ASSERT_EQ(
      std::get<OrtoolsLib::startEndPair>(routing_model.depot_config).starts,
      expected_depot_config.starts);
  ASSERT_EQ(std::get<OrtoolsLib::startEndPair>(routing_model.depot_config).ends,
            expected_depot_config.ends);
  OrtoolsLib::RoutingOptionWithCapacity expected_with_capacity{
      .capacities = {1, 2},
      .demands = {1, 2, 3},
  };

  ASSERT_TRUE(routing_model.with_capacity.has_value());
  ASSERT_EQ(routing_model.with_capacity.value().capacities,
            expected_with_capacity.capacities);
  ASSERT_EQ(routing_model.with_capacity.value().demands,
            expected_with_capacity.demands);
  OrtoolsLib::RoutingOptionWithPickupDelivery expected_with_pickup_delivery{
      .pickups_deliveries = {{1, 2}},
  };

  ASSERT_TRUE(routing_model.with_pickup_delivery.has_value());
  ASSERT_EQ(routing_model.with_pickup_delivery.value().pickups_deliveries,
            expected_with_pickup_delivery.pickups_deliveries);
  OrtoolsLib::RoutingOptionWithTimeWindow expected_with_time_window{
      .time_windows = {{{1, 2}}},
  };

  ASSERT_TRUE(routing_model.with_time_window.has_value());
  ASSERT_EQ(routing_model.with_time_window.value().time_windows,
            expected_with_time_window.time_windows);
  OrtoolsLib::RoutingOptionWithServiceTime expected_with_service_time{
      .service_time = {1, 1, 1},
  };
  ASSERT_TRUE(routing_model.with_service_time.has_value());
  ASSERT_EQ(routing_model.with_service_time.value().service_time,
            expected_with_service_time.service_time);

  ASSERT_TRUE(routing_model.with_drop_penalties.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(
      routing_model.with_drop_penalties.value().penalties));

  ASSERT_EQ(
      std::get<int64_t>(routing_model.with_drop_penalties.value().penalties),
      1);
  OrtoolsLib::RoutingOptionWithVehicleBreakTime
      expected_with_vehicle_break_time{
          .break_time = {{{1, 2}}},
      };

  ASSERT_TRUE(routing_model.with_vehicle_break_time.has_value());
  ASSERT_EQ(routing_model.with_vehicle_break_time.value().break_time,
            expected_with_vehicle_break_time.break_time);
}