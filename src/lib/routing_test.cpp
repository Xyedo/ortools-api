#include "routing.h"

#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include <vector>

const std::vector<std::vector<int64_t>> g_duration_matrix = {
    {0, 2451, 713, 1018, 1631, 1374, 2408, 213, 2571, 875, 1420, 2145, 1972},
    {2451, 0, 1745, 1524, 831, 1240, 959, 2596, 403, 1589, 1374, 357, 579},
    {713, 1745, 0, 355, 920, 803, 1737, 851, 1858, 262, 940, 1453, 1260},
    {1018, 1524, 355, 0, 700, 862, 1395, 1123, 1584, 466, 1056, 1280, 987},
    {1631, 831, 920, 700, 0, 663, 1021, 1769, 949, 796, 879, 586, 371},
    {1374, 1240, 803, 862, 663, 0, 1681, 1551, 1765, 547, 225, 887, 999},
    {2408, 959, 1737, 1395, 1021, 1681, 0, 2493, 678, 1724, 1891, 1114, 701},
    {213, 2596, 851, 1123, 1769, 1551, 2493, 0, 2699, 1038, 1605, 2300, 2099},
    {2571, 403, 1858, 1584, 949, 1765, 678, 2699, 0, 1744, 1645, 653, 600},
    {875, 1589, 262, 466, 796, 547, 1724, 1038, 1744, 0, 679, 1272, 1162},
    {1420, 1374, 940, 1056, 879, 225, 1891, 1605, 1645, 679, 0, 1017, 1200},
    {2145, 357, 1453, 1280, 586, 887, 1114, 2300, 653, 1272, 1017, 0, 504},
    {1972, 579, 1260, 987, 371, 999, 701, 2099, 600, 1162, 1200, 504, 0},
};

TEST(RoutingTest, SingleVehicleWithDepot) {
  auto responses = OrtoolsLib::Routing::builder()
                       .setDurationMatrix(g_duration_matrix)
                       .setDepotConfig(OrtoolsLib::SingleDepot{.depot = 0})
                       .build()
                       .solve();

  EXPECT_EQ(responses.size(), 1);
}

TEST(RoutingTest, SingleVehicleWithStartEndAndServiceTime) {


  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix(g_duration_matrix)
          .setDepotConfig(OrtoolsLib::startEndPair{
              .starts = {0},
              .ends = {-1},
          })
          .withServiceTime(OrtoolsLib::RoutingOptionWithServiceTime{
              .service_time = {0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
                               15}})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {0, 7, 2, 3, 9, 10, 5, 4, 12, 11, 1, 8, 6};
  EXPECT_EQ(expected_route, responses[0].route);
}

TEST(RoutingTest, SingleVehicleWithPickAndDrop) {

  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::startEndPair{
              .starts = {-1},
              .ends = {-1},
          })
          .withPickupDelivery(
              OrtoolsLib::RoutingOptionWithPickupDelivery{.pickups_deliveries =
                                                              {
                                                                  {2, 0},
                                                                  {3, 1},
                                                                  {3, 2},
                                                              }})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {3, 3, 2, 2, 0, 1};
  EXPECT_EQ(expected_route, responses[0].route);
}

TEST(RoutingTest, SingleVehicleWithPickAndDropAndDepot) {
 

  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::SingleDepot{.depot = 1})
          .withPickupDelivery(
              OrtoolsLib::RoutingOptionWithPickupDelivery{.pickups_deliveries =
                                                              {
                                                                  {2, 0},
                                                                  {3, 1},
                                                                  {3, 2},
                                                              }})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {1, 3, 3, 2, 2, 0, 1, 1};
  EXPECT_EQ(expected_route, responses[0].route);
}

TEST(RoutingTest, PickAndDropWithCapacity) {
  
  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::SingleDepot{.depot = 1})
          .withPickupDelivery(
              OrtoolsLib::RoutingOptionWithPickupDelivery{.pickups_deliveries =
                                                              {
                                                                  {2, 0},
                                                                  {3, 1},
                                                                  {3, 2},
                                                              }})
          .withCapacity(OrtoolsLib::RoutingOptionWithCapacity{
              .capacities = {40}, .demands = {5, 10, 10, 30}})
          .withDropPenalties(
              OrtoolsLib::RoutingOptionWithPenalties{.penalties{int64_t(1000)}})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {1, 2, 0, 3, 1, 1};
  EXPECT_EQ(expected_route, responses[0].route);
}

TEST(RoutingTest, WithTimeWindow) {
  
  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::startEndPair{
              .starts = {0},
              .ends = {-1},
          })
          .withTimeWindow(
              OrtoolsLib::RoutingOptionWithTimeWindow{.time_windows =
                                                          {
                                                              {{0, 40}},
                                                              {{0, 40}},
                                                              {{0, 40}},
                                                              {{0, 40}},
                                                          }})
          .withDropPenalties(
              OrtoolsLib::RoutingOptionWithPenalties{.penalties{int64_t(1000)}})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {0, 1, 2, 3};
  EXPECT_EQ(expected_route, responses[0].route);
  EXPECT_EQ(responses[0].total_duration, 11);
}

TEST(RoutingTest, WithVehicleBreakTime) {
  

  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::startEndPair{
              .starts = {0},
              .ends = {-1},
          })
          .withVehicleBreakTime(
              OrtoolsLib::RoutingOptionWithVehicleBreakTime{.break_time =
                                                                {
                                                                    {{2, 5}},
                                                                }})
          .withDropPenalties(
              OrtoolsLib::RoutingOptionWithPenalties{.penalties{int64_t(1000)}})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route = {0, 1, 2, 3};
  EXPECT_EQ(expected_route, responses[0].route);
  EXPECT_EQ(responses[0].total_duration, 14);
}

TEST(RoutingTest, OneVehicleAllConfig) {

  auto responses =
      OrtoolsLib::Routing::builder()
          .setDurationMatrix({
              {0, 1, 2, 3},
              {1, 0, 4, 5},
              {2, 4, 0, 6},
              {3, 5, 6, 0},
          })
          .setDepotConfig(OrtoolsLib::startEndPair{
              .starts = {0},
              .ends = {-1},
          })
          .withServiceTime(OrtoolsLib::RoutingOptionWithServiceTime{
              .service_time = {0, 1, 1, 1}})
          .withPickupDelivery(
              OrtoolsLib::RoutingOptionWithPickupDelivery{.pickups_deliveries =
                                                              {
                                                                  {2, 0},
                                                                  {3, 1},
                                                                  {3, 2},
                                                              }})
          .withCapacity(OrtoolsLib::RoutingOptionWithCapacity{
              .capacities = {100}, .demands = {5, 10, 10, 30}})
          .withTimeWindow(
              OrtoolsLib::RoutingOptionWithTimeWindow{.time_windows =
                                                          {
                                                              {{0, 40}},
                                                              {{10, 50}},
                                                              {{20, 60}},
                                                              {{30, 70}},
                                                          }})
          .withVehicleBreakTime(
              OrtoolsLib::RoutingOptionWithVehicleBreakTime{.break_time =
                                                                {
                                                                    {{2, 3}},
                                                                }})
          .withDropPenalties(
              OrtoolsLib::RoutingOptionWithPenalties{.penalties = {1000}})
          .build()
          .solve();

  EXPECT_EQ(responses.size(), 1);
  std::vector<int> expected_route{0, 3, 3, 2, 2, 0, 1};
  EXPECT_EQ(expected_route, responses[0].route);
  EXPECT_EQ(responses[0].total_duration, 44);
}