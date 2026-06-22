#pragma once

#include <cnoid/SceneDrawables>

namespace choreonoid_libigl {

  cnoid::SgMeshPtr booleanUnion(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanUnion(cnoid::SgNode* collisionshape1, cnoid::SgNode* collisionshape2);

  cnoid::SgMeshPtr booleanIntersect(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanIntersect(cnoid::SgNode* collisionshape1, cnoid::SgNode* collisionshape2);

  cnoid::SgMeshPtr booleanMinus(const cnoid::SgMesh* mesh1, const cnoid::SgMesh* mesh2);

  cnoid::SgShapePtr booleanMinus(cnoid::SgNode* collisionshape1, cnoid::SgNode* collisionshape2);

  cnoid::SgMeshPtr booleanXor(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2);

  cnoid::SgShapePtr booleanXor(cnoid::SgNode* collisionshape1, cnoid::SgNode* collisionshape2);
}

