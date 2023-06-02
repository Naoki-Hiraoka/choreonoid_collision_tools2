#include <choreonoid_bullet/choreonoid_bullet.h>

#include <iostream>
#include <vector>
#include <limits>
#include <cnoid/MeshExtractor>

#include <bulleteigen/bulleteigen.h>

namespace choreonoid_bullet {

  inline void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
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

  inline void addMeshes(std::vector<cnoid::SgMeshPtr>& models, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    model->getOrCreateVertices();
    model->setName(mesh->name());

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = tri[0];
      const int v1 = tri[1];
      const int v2 = tri[2];
      model->addTriangle(v0, v1, v2);
    }

    models.push_back(model);
  }

  inline cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return nullptr;

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << __PRETTY_FUNCTION__ << " meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }

  inline std::vector<cnoid::SgMeshPtr> convertToSgMeshes (const cnoid::SgNodePtr collisionshape){

    if (!collisionshape) return std::vector<cnoid::SgMeshPtr>();

    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    std::vector<cnoid::SgMeshPtr> models;
    if(meshExtractor->extract(collisionshape, [&]() { addMeshes(models,meshExtractor); })){
      //model->setName(collisionshape->name());
    }else{
      std::cerr << __PRETTY_FUNCTION__ << " meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return std::vector<cnoid::SgMeshPtr>();
    }

    return models;
  }


  std::shared_ptr<btConvexShape> convertToBulletModel(const cnoid::SgNodePtr collisionshape) {
    if(!collisionshape) return nullptr;

    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if(!model) return nullptr;

    std::vector<cnoid::Vector3> vertices;

    for (int i = 0; i < model->vertices()->size(); i ++ ) {
      vertices.push_back(model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
    }

    return bulleteigen::convertToBulletModel(vertices);
  }

  std::vector<std::shared_ptr<btConvexShape> > convertToBulletModels(const cnoid::SgNodePtr collisionshape) {
    if(!collisionshape) return std::vector<std::shared_ptr<btConvexShape> >();

    std::vector<cnoid::SgMeshPtr> models = convertToSgMeshes(collisionshape);

    std::vector<std::shared_ptr<btConvexShape> > bulletModels;

    for(int m=0;m<models.size();m++){
      if(!models[m]) continue;;
      std::vector<cnoid::Vector3> vertices;

      for (int i = 0; i < models[m]->vertices()->size(); i ++ ) {
        vertices.push_back(models[m]->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
      }

      std::shared_ptr<btConvexShape> bulletModel = bulleteigen::convertToBulletModel(vertices);
      if(bulletModel) bulletModels.push_back(bulletModel);
    }

    return bulletModels;
  }

  bool computeDistance(const std::shared_ptr<btConvexShape>& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::shared_ptr<btConvexShape>& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系. 最近傍点
                       ) {
    return bulleteigen::computeDistance(mesh1, p1, R1, mesh2, p2, R2, distance, q1, q2);
  }

  bool computeDistance(const std::vector<std::shared_ptr<btConvexShape> >& mesh1,
                       const Eigen::Vector3d& p1,
                       const Eigen::Matrix3d& R1,
                       const std::vector<std::shared_ptr<btConvexShape> >& mesh2,
                       const Eigen::Vector3d& p2,
                       const Eigen::Matrix3d& R2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系. 最近傍点
                       ) {
    if(mesh1.size() == 0 || mesh2.size() == 0) {
      distance = std::numeric_limits<double>::max();
      q1.setZero();
      q2.setZero();
      return true;
    }

    bool solved = false;
    double mindistance = std::numeric_limits<double>::max();
    Eigen::Vector3d minq1;
    Eigen::Vector3d minq2;

    double tmpdistance;
    Eigen::Vector3d tmpq1;
    Eigen::Vector3d tmpq2;
    for(int i=0;i<mesh1.size();i++){
      for(int j=0;j<mesh2.size();j++){
        if(bulleteigen::computeDistance(mesh1[i], p1, R1, mesh2[j], p2, R2, tmpdistance, tmpq1, tmpq2)){
          solved = true;
          if(tmpdistance < mindistance){
            mindistance = tmpdistance;
            minq1 = tmpq1;
            minq2 = tmpq2;
          }
        }
      }
    }

    if(!solved) return false;

    distance = mindistance;
    q1 = minq1;
    q2 = minq2;
    return true;
  }

  bool computeDistance(const cnoid::LinkPtr link1,
                       const cnoid::LinkPtr link2,
                       double& distance,
                       Eigen::Vector3d& q1,
                       Eigen::Vector3d& q2//q1,q2はlocal系. 最近傍点
                       ) {
    std::shared_ptr<btConvexShape> mesh1 = convertToBulletModel(link1->collisionShape());
    std::shared_ptr<btConvexShape> mesh2 = convertToBulletModel(link2->collisionShape());

    if(!mesh1 || !mesh2) return false;

    return computeDistance(mesh1, link1->p(), link1->R(), mesh2, link2->p(), link2->R(), distance, q1, q2);
  }
}
