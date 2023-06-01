#ifndef CHOREONOID_CDDLIB_H
#define CHOREONOID_CDDLIB_H

#include <memory>

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>

// meshはconvexである必要がある

namespace choreonoid_cddlib {
  /*
    A p = b
    dl <= C p <= du
    A,Cの各行のノルムは1 or 0
   */

  // collisionshape全体で1つのConvexShapeにする
  bool convertToFACEExpression(const cnoid::SgNodePtr collisionshape,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& A, // ? x 3. link local
                               Eigen::VectorXd& b,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               );
  // collisionshape全体で1つのConvexShapeにする
  bool convertToFACEExpression(const cnoid::SgNodePtr collisionshape,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               );

  // collisionshapeの各meshごとに1つのConvexShapeにする
  bool convertToFACEExpressions(const cnoid::SgNodePtr collisionshape,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& bs,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& dls,
                                std::vector<Eigen::VectorXd>& dus
                                );

  // collisionshapeの各meshごとに1つのConvexShapeにする
  bool convertToFACEExpressions(const cnoid::SgNodePtr collisionshape,
                                std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // ? x 3. link local
                                std::vector<Eigen::VectorXd>& dls,
                                std::vector<Eigen::VectorXd>& dus
                                );

  bool convertToFACEExpression(const Eigen::MatrixXd& V, // [v1, v2, ...]
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& A, // ? x 3. link local
                               Eigen::VectorXd& b,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               );
  bool convertToFACEExpression(const Eigen::MatrixXd& V, // [v1, v2, ...]
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& C, // ? x 3. link local
                               Eigen::VectorXd& dl,
                               Eigen::VectorXd& du
                               );

}

#endif
