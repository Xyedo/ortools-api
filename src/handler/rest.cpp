#include <drogon/HttpTypes.h>
#include <drogon/drogon.h>

#include "dtos/routingDto.h"
#include "lib/routing.h"

namespace v1 {
namespace routing {
class route : public drogon::HttpController<route> {
public:
  METHOD_LIST_BEGIN
  METHOD_ADD(route::routing, "", drogon::Post);
  METHOD_LIST_END

  void
  routing(const drogon::HttpRequestPtr &req,
          std::function<void(const drogon::HttpResponsePtr &)> &&callback) {

    RoutingDTO::RoutingModel model;
    try {
      model = RoutingDTO::parseJSON(req->getJsonObject());
    } catch (const RoutingDTO::ParseErrorElement &e) {
        auto resp = drogon::HttpResponse::newHttpJsonResponse(e.toJson());
        resp->setStatusCode(drogon::k400BadRequest);
        callback(resp);
        return;
    }

    std::vector<OrtoolsLib::RoutingResponse> response =
        OrtoolsLib::Routing::builder()
            .setDurationMatrix(std::move(model.duration_matrix))
            .setDepotConfig(std::move(model.depot_config))
            .setNumVehicles(model.num_vehicles)
            .setTimeLimit(model.time_limit)
            .withCapacity(std::move(model.with_capacity))
            .withPickupDelivery(std::move(model.with_pickup_delivery))
            .withTimeWindow(std::move(model.with_time_window))
            .withServiceTime(std::move(model.with_service_time))
            .withDropPenalties(std::move(model.with_drop_penalties))
            .withVehicleBreakTime(std::move(model.with_vehicle_break_time))
            .build()
            .solve();

    Json::Value routes;
    for (const auto &r : response) {
      Json::Value route;
      for (const auto &rout : r.route) {
        route.append(rout);
      }

      Json::Value route_duration(r.total_duration);

      Json::Value ret;
      ret["routes"] = route;
      ret["total_duration"] = route_duration;
      routes.append(ret);
    }

    Json::Value jsonResp;
    jsonResp["status"] = "success";
    jsonResp["data"] = routes;

    auto resp = drogon::HttpResponse::newHttpJsonResponse(jsonResp);
    resp->setStatusCode(drogon::k200OK);
    callback(resp);
  }
};
} // namespace routing
} // namespace v1
