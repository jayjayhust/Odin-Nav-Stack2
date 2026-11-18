/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Stair prior for the global planner.
//
// A 2D occupancy grid says nothing about which side of a flight the robot
// should keep to, and a stair is wider than the robot is long: A* will happily
// cut the corner, climb along the railing, and hand the local planner a
// start pose facing the wall. The mesh simulator fits the flights (centre,
// axis, width, and the centre of the tread at every step) and writes them out
// as "<map>.stairs.json". This header reads that file back, in the same frame
// as the .pgm next to it, and turns it into an A* cost: a multiplier that
// grows with the lateral distance from the tread centre, and is flat outside
// the flight.
//
// The file is the simulator's export verbatim:
//   { "map": "...", "res": 0.15, "origin": [x0, y0, z0],
//     "flights": [ { "id": 0, "centre": [cx, cy], "axis": [ux, uy],
//                    "half": 0.93, "length": 2.56, "rise": 1.65,
//                    "z": [zmin, zmax], "columns": 226,
//                    "spine": [ ... ], "halfAt": [ ... ], "kLo": 18 } ] }
//
// A missing or unreadable file is not an error: the planner simply runs with
// no prior, exactly as before.

#pragma once

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <string>
#include <vector>

struct StairFlight {
  double cx{0.0}, cy{0.0};   // footprint centre, map frame [m]
  double ux{1.0}, uy{0.0};   // unit axis along the run, map frame
  double half{0.5};          // half width of the footprint [m]
  double len{1.0};           // along-axis length [m]
  double zmin{0.0}, zmax{0.0};
  // The tread centre is not the footprint centre: at the foot the first steps
  // merge into the floor. The simulator writes one centre per step; a 2D grid
  // has no height to pick by, so the planner uses the mean over the flight.
  double spine_mean{0.0};    // lateral offset of the tread centre [m]
  double half_mean{0.5};     // mean half width of the walkable tread [m]
  int columns{0};
};

class StairPrior {
public:
  // Returns false when there is nothing usable; the caller then keeps
  // stairCostFactor() at exactly 1.0 for every cell.
  bool load(const std::string& path) {
    flights_.clear();
    path_.clear();
    if (path.empty()) return false;
    boost::property_tree::ptree root;
    try {
      boost::property_tree::read_json(path, root);
    } catch (const std::exception&) {
      return false;
    }
    const auto flights = root.get_child_optional("flights");
    if (!flights) return false;
    for (const auto& pair : *flights) {
      const boost::property_tree::ptree& f = pair.second;
      StairFlight s;
      const auto centre = f.get_child_optional("centre");
      const auto axis = f.get_child_optional("axis");
      if (!centre || !axis) continue;
      auto numbers = [](const boost::property_tree::ptree& node, std::vector<double>& out) {
        for (const auto& v : node) {
          try { out.push_back(v.second.get_value<double>()); } catch (...) { out.push_back(0.0); }
        }
      };
      std::vector<double> c, a;
      numbers(*centre, c);
      numbers(*axis, a);
      if (c.size() < 2 || a.size() < 2) continue;
      s.cx = c[0]; s.cy = c[1];
      const double norm = std::hypot(a[0], a[1]);
      if (norm < 1e-6) continue;
      s.ux = a[0] / norm; s.uy = a[1] / norm;
      s.half = f.get("half", 0.5);
      s.len = f.get("length", 1.0);
      s.columns = f.get("columns", 0);
      const auto z = f.get_child_optional("z");
      if (z) {
        std::vector<double> zz; numbers(*z, zz);
        if (zz.size() >= 2) { s.zmin = zz[0]; s.zmax = zz[1]; }
      }
      // spine: one lateral offset per step, nulls where the scan found nothing
      double spine_sum = 0.0, half_sum = 0.0;
      int n_spine = 0, n_half = 0;
      const auto spine = f.get_child_optional("spine");
      if (spine) for (const auto& v : *spine) {
        const std::string text = v.second.data();
        if (text.empty() || text == "null") continue;
        try { spine_sum += std::stod(text); ++n_spine; } catch (...) {}
      }
      const auto half_at = f.get_child_optional("halfAt");
      if (half_at) for (const auto& v : *half_at) {
        const std::string text = v.second.data();
        if (text.empty() || text == "null") continue;
        try { half_sum += std::stod(text); ++n_half; } catch (...) {}
      }
      s.spine_mean = n_spine ? spine_sum / n_spine : 0.0;
      s.half_mean = n_half ? half_sum / n_half : s.half;
      if (s.half <= 0.0 || s.len <= 0.0) continue;
      flights_.push_back(s);
    }
    path_ = path;
    return !flights_.empty();
  }

  bool empty() const { return flights_.empty(); }
  std::size_t size() const { return flights_.size(); }
  const std::vector<StairFlight>& flights() const { return flights_; }
  const std::string& path() const { return path_; }

  // Cost multiplier at a map-frame point: 1.0 outside every flight, and
  // 1 + weight * |offset| on it, where offset is the lateral distance from the
  // tread centre. Linear, so twice as far from the centre costs twice as much
  // more; the extra 0.30 m of slack at the edge keeps the boundary from
  // flickering between neighbouring cells.
  double costFactor(double wx, double wy, double weight) const {
    if (flights_.empty() || weight <= 0.0) return 1.0;
    double factor = 1.0;
    for (const StairFlight& f : flights_) {
      const double ox = wx - f.cx, oy = wy - f.cy;
      const double along = ox * f.ux + oy * f.uy;
      if (std::fabs(along) > f.len * 0.5 + 0.35) continue;
      const double perp = -ox * f.uy + oy * f.ux;
      const double d = std::fabs(perp - f.spine_mean);
      if (d > f.half_mean + 0.30) continue;
      factor = std::max(factor, 1.0 + weight * d);
    }
    return factor;
  }

private:
  std::vector<StairFlight> flights_;
  std::string path_;
};
