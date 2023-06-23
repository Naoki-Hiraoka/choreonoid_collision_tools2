#ifndef CHOREONOID_QHULL_H
#define CHOREONOID_QHULL_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>

namespace choreonoid_qhull{
  // 以下の場合はnullptrを返す
  // - collisionshapeがnullptrである場合
  // - meshの頂点数が4未満であるなどによってconvex hullの計算に失敗した場合
  cnoid::SgNodePtr convertToConvexHull(const cnoid::SgNodePtr collisionshape);

  void convertAllCollisionToConvexHull(cnoid::BodyPtr& robot);

  cnoid::SgShapePtr generateMeshFromConvexHull(const std::vector<Eigen::Vector3d>& vertices);
  cnoid::SgShapePtr generateMeshFromConvexHull(const Eigen::MatrixXd& vertices);
  Eigen::Matrix<double,3,Eigen::Dynamic> meshToEigen(const cnoid::SgNodePtr collisionshape);
}

#endif
