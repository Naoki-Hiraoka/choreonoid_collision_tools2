#include <choreonoid_libigl/choreonoid_libigl.h>
#include <mesh_boolean_libigl/mesh_boolean_libigl.h>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>
#include <iostream>

namespace choreonoid_libigl {
  inline bool meshToEigen(const cnoid::SgMeshPtr mesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F){
    V = Eigen::MatrixXd(3,mesh?mesh->getOrCreateVertices()->size():0); // [v1 v2 v3 v4 ..]
    F = Eigen::MatrixXi(3,mesh?mesh->numTriangles():0); // [f1 f2 f3 f4 ..]

    if(!mesh) return false;

    for(size_t i=0;i<mesh->vertices()->size();i++){
      V.col(i) = mesh->vertices()->at(i).cast<Eigen::Vector3d::Scalar>();
    }
    for(size_t i=0;i<mesh->numTriangles();i++){
      F.col(i) = mesh->triangle(i);
    }
    return true;
  }

  inline cnoid::SgMeshPtr eigenToMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F){
    cnoid::SgMeshPtr mesh(new cnoid::SgMesh);
    mesh->getOrCreateVertices()->resize(V.cols());
    mesh->setNumTriangles(F.cols());
    for(size_t i=0;i<V.cols();i++){
      mesh->vertices()->at(i) = V.col(i).cast<cnoid::Vector3f::Scalar>();
    }
    for(size_t i=0;i<F.cols();i++){
      mesh->setTriangle(i, F.col(i)[0], F.col(i)[1], F.col(i)[2]);
    }
    return mesh;
  }

  cnoid::SgMeshPtr booleanUnion(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2){
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    meshToEigen(mesh1, V1, F1);
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    meshToEigen(mesh2, V2, F2);
    Eigen::MatrixXd V3;
    Eigen::MatrixXi F3;
    mesh_boolean_libigl::boolean_union(V1, F1, V2, F2, V3, F3);
    return eigenToMesh(V3, F3);
  }

  cnoid::SgShapePtr booleanUnion(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2) {
    cnoid::MeshExtractor meshExtractor;
    cnoid::SgMeshPtr mesh1 = meshExtractor.integrate(collisionshape1);
    cnoid::SgMeshPtr mesh2 = meshExtractor.integrate(collisionshape2);
    cnoid::SgMeshPtr mesh = booleanUnion(mesh1,mesh2);
    cnoid::MeshFilter meshFilter;
    meshFilter.generateNormals(mesh,0.0);
    cnoid::SgShapePtr shape(new cnoid::SgShape);
    shape->setMesh(mesh);
    return shape;
  }

  cnoid::SgMeshPtr booleanIntersect(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2){
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    meshToEigen(mesh1, V1, F1);
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    meshToEigen(mesh2, V2, F2);
    Eigen::MatrixXd V3;
    Eigen::MatrixXi F3;
    mesh_boolean_libigl::boolean_intersect(V1, F1, V2, F2, V3, F3);
    return eigenToMesh(V3, F3);
  }

  cnoid::SgShapePtr booleanIntersect(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2){
    cnoid::MeshExtractor meshExtractor;
    cnoid::SgMeshPtr mesh1 = meshExtractor.integrate(collisionshape1);
    cnoid::SgMeshPtr mesh2 = meshExtractor.integrate(collisionshape2);
    cnoid::SgMeshPtr mesh = booleanIntersect(mesh1,mesh2);
    cnoid::MeshFilter meshFilter;
    meshFilter.generateNormals(mesh,0.0);
    cnoid::SgShapePtr shape(new cnoid::SgShape);
    shape->setMesh(mesh);
    return shape;
  }

  cnoid::SgMeshPtr booleanMinus(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2){
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    meshToEigen(mesh1, V1, F1);
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    meshToEigen(mesh2, V2, F2);
    Eigen::MatrixXd V3;
    Eigen::MatrixXi F3;
    mesh_boolean_libigl::boolean_minus(V1, F1, V2, F2, V3, F3);
    return eigenToMesh(V3, F3);
  }

  cnoid::SgShapePtr booleanMinus(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2){
    cnoid::MeshExtractor meshExtractor;
    cnoid::SgMeshPtr mesh1 = meshExtractor.integrate(collisionshape1);
    cnoid::SgMeshPtr mesh2 = meshExtractor.integrate(collisionshape2);
    cnoid::SgMeshPtr mesh = booleanMinus(mesh1,mesh2);
    cnoid::MeshFilter meshFilter;
    meshFilter.generateNormals(mesh,0.0);
    cnoid::SgShapePtr shape(new cnoid::SgShape);
    shape->setMesh(mesh);
    return shape;
  }

  cnoid::SgMeshPtr booleanXor(const cnoid::SgMeshPtr mesh1, const cnoid::SgMeshPtr mesh2){
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    meshToEigen(mesh1, V1, F1);
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    meshToEigen(mesh2, V2, F2);
    Eigen::MatrixXd V3;
    Eigen::MatrixXi F3;
    mesh_boolean_libigl::boolean_xor(V1, F1, V2, F2, V3, F3);
    return eigenToMesh(V3, F3);
  }

  cnoid::SgShapePtr booleanXor(const cnoid::SgNodePtr collisionshape1, const cnoid::SgNodePtr collisionshape2){
    cnoid::MeshExtractor meshExtractor;
    cnoid::SgMeshPtr mesh1 = meshExtractor.integrate(collisionshape1);
    cnoid::SgMeshPtr mesh2 = meshExtractor.integrate(collisionshape2);
    cnoid::SgMeshPtr mesh = booleanXor(mesh1,mesh2);
    cnoid::MeshFilter meshFilter;
    meshFilter.generateNormals(mesh,0.0);
    cnoid::SgShapePtr shape(new cnoid::SgShape);
    shape->setMesh(mesh);
    return shape;
  }

}
