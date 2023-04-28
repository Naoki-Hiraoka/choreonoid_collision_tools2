#ifndef CHOREONOID_QHULL_H
#define CHOREONOID_QHULL_H

#include <cnoid/Body>

namespace choreonoid_qhull{
  cnoid::SgNodePtr convertToConvexHull(const cnoid::SgNodePtr collisionshape);

  void convertAllCollisionToConvexHull(cnoid::BodyPtr& robot);

  cnoid::SgNodePtr generateMeshFromConvexHull(const std::vector<Eigen::Vector3d> vertices);
}

#endif
