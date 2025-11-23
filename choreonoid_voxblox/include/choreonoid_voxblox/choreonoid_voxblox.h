#ifndef CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H
#define CHOREONOID_VOXBLOX_CHOREONOID_BOXBLOX_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <cnoid/RangeCamera>
#include <voxblox/mesh/mesh.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>

namespace choreonoid_voxblox {
  cnoid::SgMeshPtr convertToChoreonoidModel(const std::shared_ptr<const voxblox::Mesh>& mesh);
  cnoid::BodyPtr convertToChoreonoidBody(const std::shared_ptr<voxblox::TsdfMap>& tsdf_map);

  bool insertToTsdf(const std::vector<cnoid::BodyPtr>& bodies, // input
                    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdfIntegrator, // in out
                    double ray = 0.1, // input
                    double resolution = 0.01, // input
                    cnoid::Isometry3 mapOrigin = cnoid::Isometry3::Identity() // input
                    );
  bool insertToTsdf(const cnoid::RangeCameraPtr camera, // input
                    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdfIntegrator, // in out
                    cnoid::Isometry3 mapOrigin // input
                    );
  class calcEsdfParam{
  public:
    double voxel_size = 0.02;
    double default_distance = 0.5;
    int resolution = 200;
    double maxDistance = 1.0;
    double min_x = -1.0;
    double max_x = 1.0;
    double min_y = -1.0;
    double max_y = 1.0;
    double min_z = -1.0;
    double max_z = 1.0;
    double step = 0.3;
  };
  bool calcEsdf(const std::vector<cnoid::BodyPtr>& obstacles, // input
                std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> >& collisionModels, // input
                std::shared_ptr<voxblox::TsdfMap>& tsdf_map, // output
                std::shared_ptr<voxblox::EsdfMap>& esdf_map, // output
                const calcEsdfParam& param
                );
  bool loadOrCalcEsdf(const std::string& vxblxFileName,
                      const std::vector<cnoid::BodyPtr>& obstacles, // input
                      std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> >& collisionModels, // input
                      std::shared_ptr<voxblox::TsdfMap>& tsdf_map, // output
                      std::shared_ptr<voxblox::EsdfMap>& esdf_map, // output
                      const calcEsdfParam& param
                      );
};

#endif
