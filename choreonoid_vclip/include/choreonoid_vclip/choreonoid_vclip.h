#ifndef CHOREONOID_VCLIP_H
#define CHOREONOID_VCLIP_H

#include <memory>

#include <cnoid/Body>
#include <cnoid/SceneDrawables>

namespace Vclip{
  class Polyhedron;
}

// meshはconvexである必要がある

namespace choreonoid_vclip {
  std::shared_ptr<Vclip::Polyhedron> convertToVClipModel(const cnoid::SgNodePtr collisionshape);

  bool computeDistance(const std::shared_ptr<Vclip::Polyhedron>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<Vclip::Polyhedron>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2);//q1,q2はlocal系. 最近傍点

  // 内部で上2つの関数を呼ぶ. meshの形状が変わらないなら、convertToVClipModelを毎回呼ぶのは無駄なので、上２つの関数を手動で組み合わせた方が速い
  bool computeDistance(const cnoid::LinkPtr link1,
                       const cnoid::LinkPtr link2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2);//q1,q2はlocal系. 最近傍点
}

#endif
