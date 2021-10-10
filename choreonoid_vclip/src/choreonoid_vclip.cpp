#include <choreonoid_vclip/choreonoid_vclip.h>

#include <iostream>
#include <vector>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>

#include <vclipeigen/vclipeigen.h>

namespace choreonoid_vclip {
  void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->getOrCreateVertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }

  std::shared_ptr<Vclip::Polyhedron> convertToVClipModel(const cnoid::SgNodePtr collisionshape) {
    if(!collisionshape) return nullptr;

    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if(!model) return nullptr;

    std::vector<cnoid::Vector3> vertices;

    for (int i = 0; i < model->vertices()->size(); i ++ ) {
      vertices.push_back(model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
    }

    return vclipeigen::convertToVClipModel(vertices);
  }

  bool computeDistance(const std::shared_ptr<Vclip::Polyhedron>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<Vclip::Polyhedron>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系. 最近傍点
                       ) {
    return vclipeigen::computeDistance(mesh1, p1, R1, mesh2, p2, R2, distance, q1, q2);
  }

  bool computeDistance(const cnoid::LinkPtr link1,
                       const cnoid::LinkPtr link2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系. 最近傍点
                       ) {
    std::shared_ptr<Vclip::Polyhedron> mesh1 = convertToVClipModel(link1->collisionShape());
    std::shared_ptr<Vclip::Polyhedron> mesh2 = convertToVClipModel(link2->collisionShape());

    if(!mesh1 || mesh2) return false;

    return computeDistance(mesh1, link1->p(), link1->R(), mesh2, link2->p(), link2->R(), distance, q1, q2);
  }
}
