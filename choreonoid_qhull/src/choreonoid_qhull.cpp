#include <choreonoid_qhull/choreonoid_qhull.h>

#include <iostream>

#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>

#include <qhulleigen/qhulleigen.h>

namespace choreonoid_qhull{
  cnoid::SgNodePtr convertToConvexHull(const cnoid::SgNodePtr collisionshape) {
    if(!collisionshape) return nullptr;
    // qhull
    Eigen::MatrixXd vertices = meshToEigen(collisionshape);
    cnoid::SgShapePtr ret = generateMeshFromConvexHull(vertices);
    if(ret) ret->setName(collisionshape->name());
    return ret;
  }

  void convertAllCollisionToConvexHull(cnoid::BodyPtr& robot){
    for(size_t i=0;i<robot->numLinks();i++){
      cnoid::SgNodePtr coldetModel = convertToConvexHull(robot->link(i)->collisionShape());
      if(coldetModel){
        std::vector<cnoid::SgNodePtr> shapes;
        {
          cnoid::SgGroup* group = robot->link(i)->shape();
          for(int j=0;j<group->numChildObjects();j++){
            shapes.emplace_back(group->child(j));
          }
        }
        robot->link(i)->clearShapeNodes();
        for(int j=0;j<shapes.size();j++){
          robot->link(i)->addVisualShapeNode(shapes[j]);
        }
        robot->link(i)->addCollisionShapeNode(coldetModel);
      }else{
        std::cerr << __PRETTY_FUNCTION__ << " convex hull " << robot->link(i)->name() << " fail" << std::endl;
      }
    }
  }

  cnoid::SgShapePtr generateMeshFromConvexHull(const std::vector<Eigen::Vector3d>& vertices_) {
    Eigen::MatrixXd vertices(3,vertices_.size());
    for(size_t i=0;i<vertices_.size();i++){
      vertices.col(i) = vertices_[i];
    }
    return generateMeshFromConvexHull(vertices);
  }

  cnoid::SgShapePtr generateMeshFromConvexHull(const Eigen::MatrixXd& vertices) {
    // qhull
    Eigen::MatrixXd hull;
    std::vector<std::vector<int> > faces;
    if(!qhulleigen::convexhull(vertices,hull,faces)) return nullptr;

    cnoid::SgMeshPtr coldetModel(new cnoid::SgMesh);

    coldetModel->getOrCreateVertices()->resize(hull.cols());
    coldetModel->setNumTriangles(faces.size());

    for(size_t i=0;i<hull.cols();i++){
      coldetModel->vertices()->at(i) = hull.col(i).cast<cnoid::Vector3f::Scalar>();
    }

    for(size_t i=0;i<faces.size();i++){
      coldetModel->setTriangle(i, faces[i][0], faces[i][1], faces[i][2]);
    }

    cnoid::SgShapePtr ret(new cnoid::SgShape);
    ret->setMesh(coldetModel);
    return ret;
  }

  Eigen::Matrix<double,3,Eigen::Dynamic> meshToEigen(const cnoid::SgNodePtr collisionshape){
    if(!collisionshape) return Eigen::MatrixXd(3,0);
    cnoid::MeshExtractor meshExtractor;
    cnoid::SgMeshPtr model = meshExtractor.integrate(collisionshape);

    if (!model || model->getOrCreateVertices()->size()==0) return Eigen::MatrixXd(3,0);

    Eigen::Matrix<double,3,Eigen::Dynamic> vertices(3,model->vertices()->size());
    for(size_t i=0;i<model->vertices()->size();i++){
      vertices.col(i) = model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>();
    }
    return vertices;
  }
}
