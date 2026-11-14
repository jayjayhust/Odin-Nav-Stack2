#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <string>

namespace floor_manager {

// Load a map_server-compatible YAML + PGM pair into an OccupancyGrid.
//
// Only the keys ONS2 needs are parsed (image, resolution, origin, negate,
// occupied_thresh, free_thresh). Row order is flipped so that grid row 0 is
// the bottom row of the image, matching map_server semantics.
bool loadOccupancyGridFromYaml(const std::string& yaml_path,
                               nav_msgs::OccupancyGrid& grid,
                               std::string& error);

}  // namespace floor_manager
