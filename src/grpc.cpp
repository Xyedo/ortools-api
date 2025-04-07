
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <routing-proto/routing.grpc.pb.h>

#include <variant>

#include "lib/routing.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;
using std::chrono::system_clock;

namespace grpc {
class OrtoolsImpl final : public routing::OrtoolsService::Service {
  Status Routing(ServerContext *context,
                 const routing::RoutingRequest *const request,
                 routing::RoutingResponse *response) override {
    vector<vector<int64_t>> duration_matrix(request->durationmatrix().size());
    for (const auto &row : request->durationmatrix()) {
      vector<int64_t> row_vector(row.value().begin(), row.value().end());
      duration_matrix.push_back(row_vector);
    }

    variant<ortoolsRouting::SingleDepot, ortoolsRouting::startEndPair>
        depot_config;

    if (request->has_depot()) {
      depot_config.emplace<ortoolsRouting::SingleDepot>(
          ortoolsRouting::SingleDepot{.depot = request->depot()});
    }
    if (request->has_startend()) {
      depot_config.emplace<ortoolsRouting::startEndPair>(
          ortoolsRouting::startEndPair{
              .starts = vector<int32_t>(request->startend().start().begin(),
                                        request->startend().start().end()),
              .ends = vector<int32_t>(request->startend().end().begin(),
                                      request->startend().end().end()),
          });
    }
    optional<ortoolsRouting::RoutingOptionWithCapacity> with_capacity;
    if (request->has_withcapacity()) {
      with_capacity.emplace(ortoolsRouting::RoutingOptionWithCapacity{
          .capacities =
              vector<int64_t>(request->withcapacity().vehiclecapacity().begin(),
                              request->withcapacity().vehiclecapacity().end()),
          .demands = vector<int64_t>(request->withcapacity().demands().begin(),
                                     request->withcapacity().demands().end()),
      });
    }

    optional<ortoolsRouting::RoutingOptionWithPickupDelivery>
        with_pickup_delivery;
    if (request->has_withpickupanddeliveries()) {
      vector<pair<int32_t, int32_t>> pickups_deliveries;
      for (const auto &pickup_delivery :
           request->withpickupanddeliveries().pickupdrops()) {
        pickups_deliveries.emplace_back(pickup_delivery.a(),
                                        pickup_delivery.b());
      }
      with_pickup_delivery.emplace(
          ortoolsRouting::RoutingOptionWithPickupDelivery{
              .pickups_deliveries = move(pickups_deliveries),
          });
    }

    optional<ortoolsRouting::RoutingOptionWithTimeWindow> with_time_window;
    if (request->has_withtimewindows()) {
      vector<pair<int64_t, int64_t>> time_windows;
      for (const auto &time_window : request->withtimewindows().timewindows()) {
        time_windows.emplace_back(time_window.a(), time_window.b());
      }

      with_time_window.emplace(ortoolsRouting::RoutingOptionWithTimeWindow{
          .time_windows = move(time_windows),
      });
    }
    optional<ortoolsRouting::RoutingOptionWithServiceTime> with_service_time;
    if (request->has_withservicetime()) {
      with_service_time.emplace(ortoolsRouting::RoutingOptionWithServiceTime{
          .service_time =
              vector<int64_t>(request->withservicetime().servicetime().begin(),
                              request->withservicetime().servicetime().end()),
      });
    }

    const vector<ortoolsRouting::RoutingResponse> resp =
        ortoolsRouting::Routing(ortoolsRouting::RoutingConfig{
            .duration_matrix = move(duration_matrix),
            .depot_config = move(depot_config),
            .num_vehicles = request->numvehicles(),
            .time_limit = request->apitimelimit(),
            .with_capacity = move(with_capacity),
            .with_pickup_delivery = move(with_pickup_delivery),
            .with_time_window = move(with_time_window),
            .with_service_time = move(with_service_time)});
    for (const auto &r : resp) {
      auto *routes = response->add_routes();
      for (const auto &route : r.route) {
        routes->add_route(route);
      }
      routes->set_totalduration(r.total_duration);
    }

    return Status::OK;
  }
};

void RunServer() {
  std::string server_address("0.0.0.0:50051");
  OrtoolsImpl service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}
}  // namespace grpc
