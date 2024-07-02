#ifndef CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H
#define CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <voxblox/mesh/mesh.h>

namespace choreonoid_voxblox {
  cnoid::SgMeshPtr convertToChoreonoidModel(const std::shared_ptr<const voxblox::Mesh>& mesh);
};

#endif
