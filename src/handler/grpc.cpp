
#include <grpc/grpc.h>
#include <grpcpp/server_builder.h>
#include <routing-proto/routing.grpc.pb.h>

#include <optional>
#include <variant>

#include "dtos/routingDto.h"
#include "lib/routing.h"

namespace grpcHandler {
class OrtoolsImpl final : public routing::OrtoolsService::Service {
  grpc::Status Routing(grpc::ServerContext *context,
                       const routing::RoutingRequest *const request,
                       routing::RoutingResponse *const response) override {
    const RoutingDTO::RoutingModel routing_model =
        RoutingDTO::intoEntity(request);

    const std::vector<OrtoolsLib::RoutingResponse> resp =
        OrtoolsLib::Routing::builder()
            .setDurationMatrix(std::move(routing_model.duration_matrix))
            .setDepotConfig(std::move(routing_model.depot_config))
            .setNumVehicles(routing_model.num_vehicles)
            .setTimeLimit(routing_model.time_limit)
            .withCapacity(std::move(routing_model.with_capacity))
            .withPickupDelivery(std::move(routing_model.with_pickup_delivery))
            .withTimeWindow(std::move(routing_model.with_time_window))
            .withServiceTime(std::move(routing_model.with_service_time))
            .withDropPenalties(std::move(routing_model.with_drop_penalties))
            .withVehicleBreakTime(
                std::move(routing_model.with_vehicle_break_time))
            .build()
            .solve();

    for (const auto &r : resp) {
      auto *routes = response->add_routes();
      for (const auto &route : r.route) {
        routes->add_route(route);
      }
      routes->set_totalduration(r.total_duration);
    }

    return grpc::Status::OK;
  }
};

void RunServer() {
  std::string server_address("0.0.0.0:50051");
  OrtoolsImpl service;

  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}
}  // namespace grpcHandler
