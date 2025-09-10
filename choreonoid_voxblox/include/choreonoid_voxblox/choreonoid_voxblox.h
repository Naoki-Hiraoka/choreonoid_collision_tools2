#ifndef CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H
#define CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <voxblox/mesh/mesh.h>
#include <voxblox/integrator/tsdf_integrator.h>

namespace choreonoid_voxblox {
  cnoid::SgMeshPtr convertToChoreonoidModel(const std::shared_ptr<const voxblox::Mesh>& mesh);

  bool insertToTsdf(const std::vector<cnoid::BodyPtr>& bodies, // input
                    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdfIntegrator, // in out
                    double ray = 0.1, // input
                    double resolution = 0.01, // input
                    cnoid::Isometry3 mapOrigin = cnoid::Isometry3::Identity() // input
                    );
};

#endif
