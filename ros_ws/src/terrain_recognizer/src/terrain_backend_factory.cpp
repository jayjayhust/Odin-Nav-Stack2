// Copyright 2026 Manifold Tech Ltd.
//
// Backend registry of the terrain-recognizer slot.
//
// The map lives in a function-local static so a backend's static registrar may
// run at any point during static initialisation without touching an
// uninitialised registry.
#include "terrain_recognizer/terrain_backend.h"

#include <algorithm>
#include <map>

namespace terrain_recognizer {
namespace {

std::map<std::string, TerrainBackendFactory>& registry() {
  static std::map<std::string, TerrainBackendFactory> instance;
  return instance;
}

}  // namespace

void registerTerrainBackend(const std::string& name, TerrainBackendFactory factory) {
  registry()[name] = factory;
}

TerrainBackendPtr createTerrainBackend(const std::string& name) {
  const auto it = registry().find(name);
  if (it == registry().end() || it->second == nullptr) return nullptr;
  return it->second();
}

std::vector<std::string> availableTerrainBackends() {
  std::vector<std::string> names;
  names.reserve(registry().size());
  for (const auto& entry : registry()) names.push_back(entry.first);
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace terrain_recognizer
