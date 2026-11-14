// Copyright 2026 Manifold Tech Ltd.
//
// thin extension of unitree_sdk2's Go2 SportClient.
//
// Why this exists: the sport_api.hpp published in unitree_sdk2 stops at 1036 /
// 1063 and does not list SwitchGait or WalkStair, while the Go2 firmware does
// implement them - the ROS2 stack exposes them as /switch_gait and /mode
// "walkstair". Client::Call and Client::RegistApi are protected, so a subclass
// reaches the ids directly instead of inventing a convenience method.
//
// Ids and payload shape are confirmed from the Go2 sport client that ship with
// unitree_sdk2_python, and cross-checked against unitree_sdk2_rs, which labels
// `api 1011 SwitchGait -> {"data": ..}`:
//
//   1011  SwitchGait    {"data": int}   0 idle, 1 trot, 2 trot-running,
//                                       3 forward climbing, 4 reverse climbing
//   1049  WalkStair     {"data": bool}  the switch behind /mode "walkstair"
//
// Both are ROS parameters so an operator on different firmware can override
// them; -1 means "not configured, do not call".
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
    const std::string param = std::string("{\"data\":") + std::to_string(gait) + "}";
    std::string data;
    return Call(api_id, param, data);
  }

  // The firmware switch that the ROS2 stack exposes as /mode "walkstair".
  // Takes a bool, not a mode string: the string form belongs to the ROS2
  // service wrapper, not to the SDK rpc.
  int32_t WalkStairById(int32_t api_id, bool on) {
    if (api_id < 0) return -1;
    RegistApi(api_id, 0);
    const std::string param =
        std::string("{\"data\":") + (on ? "true" : "false") + "}";
    std::string data;
    return Call(api_id, param, data);
  }
};

}  // namespace stair_gait

#endif  // STAIR_GAIT_EXT_SPORT_CLIENT_H
