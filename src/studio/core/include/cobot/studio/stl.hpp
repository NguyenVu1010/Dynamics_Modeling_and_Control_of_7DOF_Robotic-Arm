#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include "cobot/common/result.hpp"

// STL meshes of the 3D view (feature studio-3d, ADR-021), binary or ASCII. No Qt.
namespace cobot::studio {

// One position and one normal per vertex, 9 floats per triangle each (the layout of a GPU vertex
// buffer). Units of the file (metres for src/sim/meshes).
struct TriangleMesh {
  std::vector<float> positions, normals;
  std::size_t triangles = 0;
};

// A binary file is 84 + 50 * count bytes; a file starting with "solid" that is not is read as
// ASCII. Errors name the file: truncated, count/size mismatch, bad ASCII syntax, non-finite values.
common::Result<TriangleMesh> load_stl(const std::filesystem::path& file);

}  // namespace cobot::studio
