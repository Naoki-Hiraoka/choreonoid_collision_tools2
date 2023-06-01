#include <choreonoid_cddlib/choreonoid_cddlib.h>

#include <iostream>
#include <vector>
#include <limits>
#include <cnoid/MeshExtractor>

#include <cddeigen/cddeigen.h>

namespace choreonoid_cddlib {

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
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
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
      std::cerr << "[convertToSgMeshes] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return std::vector<cnoid::SgMeshPtr>();
    }

    return models;
  }

  // collisionshape全体で1つのConvexShapeにする
  bool convertToFACEExpression(const cnoid::SgNodePtr collisionshape,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& A, // ? x 3. link local
                               Eigen::VectorXd& b,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               ){
    if(!collisionshape) return false;

    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if(!model) return false;

    std::vector<cnoid::Vector3> vertices;

    for (int i = 0; i < model->vertices()->size(); i ++ ) {
      vertices.push_back(model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
    }

    return convertToFACEExpression(vertices, A, b, C, dl, du);
  }

  // collisionshape全体で1つのConvexShapeにする
  bool convertToFACEExpression(const cnoid::SgNodePtr collisionshape,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               ){
    if(!collisionshape) return false;

    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if(!model) return false;

    std::vector<cnoid::Vector3> vertices;

    for (int i = 0; i < model->vertices()->size(); i ++ ) {
      vertices.push_back(model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
    }

    return convertToFACEExpression(vertices, C, dl, du);
  }

  // collisionshapeの各meshごとに1つのConvexShapeにする
  bool convertToFACEExpressions(const cnoid::SgNodePtr collisionshape,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& bs,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& dls,
                                std::vector<Eigen::VectorXd>& dus
                                ){
    if(!collisionshape) return false;

    std::vector<cnoid::SgMeshPtr> models = convertToSgMeshes(collisionshape);

    As.clear();
    bs.clear();
    Cs.clear();
    dls.clear();
    dus.clear();
    for(int m=0;m<models.size();m++){
      if(!models[m]) continue;
      std::vector<cnoid::Vector3> vertices;

      for (int i = 0; i < models[m]->vertices()->size(); i ++ ) {
        vertices.push_back(models[m]->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
      }

      Eigen::SparseMatrix<double,Eigen::RowMajor> A;
      Eigen::VectorXd b;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C;
      Eigen::VectorXd dl;
      Eigen::VectorXd du;

      if(convertToFACEExpression(vertices, A, b, C, dl, du)){
        As.push_back(A);
        bs.push_back(b);
        Cs.push_back(C);
        dls.push_back(dl);
        dus.push_back(du);
      }else{
        As.push_back(Eigen::SparseMatrix<double,Eigen::RowMajor>(0,3));
        bs.push_back(Eigen::VectorXd(0));
        Cs.push_back(Eigen::SparseMatrix<double,Eigen::RowMajor>(0,3));
        dls.push_back(Eigen::VectorXd(0));
        dus.push_back(Eigen::VectorXd(0));
      }
    }
    return true;
  }

  // collisionshapeの各meshごとに1つのConvexShapeにする
  bool convertToFACEExpressions(const cnoid::SgNodePtr collisionshape,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& dls,
                                std::vector<Eigen::VectorXd>& dus
                                ){
    if(!collisionshape) return false;

    std::vector<cnoid::SgMeshPtr> models = convertToSgMeshes(collisionshape);

    Cs.clear();
    dls.clear();
    dus.clear();
    for(int m=0;m<models.size();m++){
      if(!models[m]) continue;
      std::vector<cnoid::Vector3> vertices;

      for (int i = 0; i < models[m]->vertices()->size(); i ++ ) {
        vertices.push_back(models[m]->vertices()->at(i).cast<Eigen::Vector3d::Scalar>());
      }

      Eigen::SparseMatrix<double,Eigen::RowMajor> C;
      Eigen::VectorXd dl;
      Eigen::VectorXd du;

      if(convertToFACEExpression(vertices, C, dl, du)){
        Cs.push_back(C);
        dls.push_back(dl);
        dus.push_back(du);
      }else{
        Cs.push_back(Eigen::SparseMatrix<double,Eigen::RowMajor>(0,3));
        dls.push_back(Eigen::VectorXd(0));
        dus.push_back(Eigen::VectorXd(0));
      }
    }
    return true;
  }

  bool convertToFACEExpression(const std::vector<Eigen::Vector3d>& V, // [v1, v2, ...]
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& A, // ? x 3. link local
                               Eigen::VectorXd& b,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               ){
    Eigen::VectorXd V_(V.size(),3);
    for(int i=0;i<V.size();i++) V_.col(i) = V[i];

    const Eigen::MatrixXd R_nonneg(3,0);
    const Eigen::MatrixXd R_free(3,0);

    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;
    /*
      INPUT:
        x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
      OUTPUT:
        A_eq   x + b_eq    = 0
        A_ineq x + b_ineq >= 0
    */
    if(!cddeigen::VtoH(V_,
                       R_nonneg,
                       R_free,
                       A_eq,
                       b_eq,
                       A_ineq,
                       b_ineq
                       )) return false;

    // 各行のnormを1にする
    for(int i=0;i<A_eq.rows();i++){
      double norm = A_eq.row(i).norm();
      if(norm > 0) {
        A_eq.row(i) /= norm;
        b_eq[i] /= norm;
      }
    }
    for(int i=0;i<A_ineq.rows();i++){
      double norm = A_ineq.row(i).norm();
      if(norm > 0) {
        A_ineq.row(i) /= norm;
        b_ineq[i] /= norm;
      }
    }

    A.resize(A_eq.rows(), 3);
    b.resize(b_eq.size());
    for(int i=0;i<A_eq.rows();i++){
      for(int j=0;j<3;j++) A.coeffRef(i,j) = A_eq(i,j);
      b[i] = -b_eq(i);
    }
    C.resize(A_ineq.rows(),3);
    dl.resize(A_ineq.rows());
    du.resize(A_ineq.rows());
    for(int i=0;i<A_ineq.rows();i++){
      for(int j=0;j<3;j++) C.coeffRef(i,j) = A_ineq(i,j);
      dl[i] = -b_ineq(i);
      du[i] = 1e10;
    }
    return true;
  }

  bool convertToFACEExpression(const std::vector<Eigen::Vector3d>& V, // [v1, v2, ...]
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               ){
    Eigen::VectorXd V_(V.size(),3);
    for(int i=0;i<V.size();i++) V_.col(i) = V[i];

    const Eigen::MatrixXd R_nonneg(3,0);
    const Eigen::MatrixXd R_free(3,0);

    Eigen::MatrixXd A_eq;
    Eigen::VectorXd b_eq;
    Eigen::MatrixXd A_ineq;
    Eigen::VectorXd b_ineq;
    /*
      INPUT:
        x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
      OUTPUT:
        A_eq   x + b_eq    = 0
        A_ineq x + b_ineq >= 0
    */
    if(!cddeigen::VtoH(V_,
                       R_nonneg,
                       R_free,
                       A_eq,
                       b_eq,
                       A_ineq,
                       b_ineq
                       )) return false;

    // 各行のnormを1にする
    for(int i=0;i<A_eq.rows();i++){
      double norm = A_eq.row(i).norm();
      if(norm > 0) {
        A_eq.row(i) /= norm;
        b_eq[i] /= norm;
      }
    }
    for(int i=0;i<A_ineq.rows();i++){
      double norm = A_ineq.row(i).norm();
      if(norm > 0) {
        A_ineq.row(i) /= norm;
        b_ineq[i] /= norm;
      }
    }

    C.resize(A_eq.rows()+A_ineq.rows(),3);
    dl.resize(A_eq.rows()+A_ineq.rows());
    du.resize(A_eq.rows()+A_ineq.rows());
    for(int i=0;i<A_eq.rows();i++){
      for(int j=0;j<3;j++) C.coeffRef(i,j) = A_eq(i,j);
      dl[i] = -b_eq(i);
      du[i] = -b_eq(i);
    }
    for(int i=0;i<A_ineq.rows();i++){
      for(int j=0;j<3;j++) C.coeffRef(A_eq.rows()+i,j) = A_ineq(i,j);
      dl[A_eq.rows()+i] = -b_ineq(i);
      du[A_eq.rows()+i] = 1e10;
    }
    return true;
  }

}
