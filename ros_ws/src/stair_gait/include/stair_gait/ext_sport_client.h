// Copyright 2026 Manifold Tech Ltd.
//
// Thin extension of unitree_sdk2's Go2 SportClient.
//
// Why this exists: the public sport_api.hpp of unitree_sdk2 does not publish
// the API ids for SwitchGait / WalkStair, while the Go2 firmware and the ROS2
// go2_interfaces do expose them (/switch_gait with FORWARD_CLIMBING_MODE=3 and
// REVERSE_CLIMBING_MODE=4, /mode with "walkstair"). Client::Call and
// Client::RegistApi are protected, so a subclass can reach those ids without
// inventing a convenience method. The id itself is never hard-coded here: it
// comes from a ROS parameter the operator fills in from the SDK or firmware
// they actually run.
#ifndef STAIR_GAIT_EXT_SPORT_CLIENT_H
#define STAIR_GAIT_EXT_SPORT_CLIENT_H

#include <string>

#include <unitree/robot/go2/sport/sport_client.hpp>

namespace stair_gait {

class ExtSportClient : public unitree::robot::go2::SportClient {
 public:
  ExtSportClient() : unitree::robot::go2::SportClient(false) {}

  // gait: 0 idle, 1 trot, 2 trot running, 3 forward climbing, 4 reverse climbing
  int32_t SwitchGaitById(int32_t api_id, int32_t gait) {
    if (api_id < 0) return -1;
    RegistApi(api_id, 0);
    const std::string param = std::string("{\"gait\":") + std::to_string(gait) + "}";
    std::string data;
    return Call(api_id, param, data);
  }

  // mode string form used by the ROS2 /mode service ("walkstair", "freewalk", ...)
  int32_t SetModeById(int32_t api_id, const std::string& mode) {
    if (api_id < 0) return -1;
    RegistApi(api_id, 0);
    const std::string param = "{\"mode\":\"" + mode + "\"}";
    std::string data;
    return Call(api_id, param, data);
  }
};

}  // namespace stair_gait

#endif  // STAIR_GAIT_EXT_SPORT_CLIENT_H
