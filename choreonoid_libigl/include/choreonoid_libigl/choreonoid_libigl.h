#pragma once

#include <cnoid/SceneDrawables>

namespace choreonoid_libigl {

  cnoid::SgMeshPtr booleanUnion(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanUnion(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2);

  cnoid::SgMeshPtr booleanIntersect(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanIntersect(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2);

  cnoid::SgMeshPtr booleanMinus(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanMinus(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2);

  cnoid::SgMeshPtr booleanXor(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanXor(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2);
}

