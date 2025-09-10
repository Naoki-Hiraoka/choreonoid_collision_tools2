#include <choreonoid_voxblox/choreonoid_voxblox.h>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>


namespace choreonoid_voxblox {
  // collisionshape全体で1つのmeshにする
  cnoid::SgMeshPtr convertToChoreonoidModel(const std::shared_ptr<const voxblox::Mesh>& mesh) {
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    model->getOrCreateVertices();
    model->vertices()->reserve(mesh->vertices.size());
    model->getOrCreateNormals();
    model->normals()->reserve(mesh->vertices.size());
    model->getOrCreateColors();
    model->colors()->reserve(mesh->vertices.size());
    for(int i=0;i<mesh->vertices.size();i++){
      model->vertices()->push_back(mesh->vertices[i]);
      model->normals()->push_back(mesh->normals[i]);
      model->colors()->push_back(cnoid::Vector3f(mesh->colors[i].r/255.0,mesh->colors[i].g/255.0,mesh->colors[i].b/255.0));
    }
    for(int i=0; i+2 <mesh->indices.size(); i+=3){
      model->addTriangle(mesh->indices[i],
                         mesh->indices[i+1],
                         mesh->indices[i+2]);
      model->normalIndices().push_back(mesh->indices[i]);
      model->normalIndices().push_back(mesh->indices[i+1]);
      model->normalIndices().push_back(mesh->indices[i+2]);
      model->colorIndices().push_back(mesh->indices[i]);
      model->colorIndices().push_back(mesh->indices[i+1]);
      model->colorIndices().push_back(mesh->indices[i+2]);
    }
    return model;
  }


  inline std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > getSurfaceVerticesAndNormals(cnoid::LinkPtr link, float resolution, float minangle) {
    // 1つのvertexを取得したら、resolutionのサイズの同じ立方体の中にありかつ法線がminangle以下の他のvertexは取得しない
    // faceが巨大な場合、faceの内部の点をresolutionの間隔でサンプリングして取得する

    cnoid::MeshExtractor meshExtractor;
    cnoid::MeshFilter meshFilter;

    std::vector<std::pair<cnoid::Vector3, cnoid::Vector3> > vertices;
    cnoid::SgMeshPtr mesh = meshExtractor.integrate(link->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      meshFilter.generateNormals(mesh,M_PI,true);
      mesh->updateBoundingBox();
      cnoid::BoundingBoxf bbx = mesh->boundingBox();
      cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
      std::vector<std::vector<std::vector<std::vector<cnoid::Vector3f> > > > bin; // normalを入れる
      bin.resize(int(bbxSize[0]/resolution)+1);
      for(int x=0;x<bin.size();x++){
        bin[x].resize(int(bbxSize[1]/resolution)+1);
        for(int y=0;y<bin[x].size();y++){
          bin[x][y].resize(int(bbxSize[2]/resolution)+1);
        }
      }

      for(int j=0;j<mesh->numTriangles();j++){
        cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
        cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
        cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
        //cnoid::Vector3f normal = mesh->normals()->at(mesh->normalIndices()[j*3]); // 隣のfaceの影響を受けてしまう?
        cnoid::Vector3f normal; // linkの外側に向かう方向
        {
          cnoid::Vector3f dir = (v1 - v0).cross(v2 - v0);
          if(dir.norm()==0) continue;
          normal = dir.normalized();
        }

        float l1 = (v1 - v0).norm();
        float l2 = (v2 - v0).norm();
        cnoid::Vector3f d1 = (v1 - v0).normalized();
        cnoid::Vector3f d2 = (v2 - v0).normalized();

        for(float m=0;;){
          float n_max = (l1==0)? l2 : l2*(1-m/l1);
          for(float n=0;;){
            cnoid::Vector3f v = v0 + d1 * m + d2 * n;
            int x = int((v[0] - bbx.min()[0])/resolution);
            int y = int((v[1] - bbx.min()[1])/resolution);
            int z = int((v[2] - bbx.min()[2])/resolution);

            bool exists = false;
            for(int s=0;s<bin[x][y][z].size();s++){
              if(minangle >= std::acos(std::min(1.0f,(std::max(-1.0f,normal.dot(bin[x][y][z][s])))))){
                exists = true;
                break;
              }
            }
            if(!exists){
              bin[x][y][z].push_back(normal);
              vertices.emplace_back(v.cast<double>(), normal.cast<double>());
            }

            if(n>= n_max) break;
            else n = std::min(n+resolution, n_max);
          }

          if(m>=l1) break;
          else m = std::min(m+resolution, l1);
        }
      }
    }
    return vertices;
  }

  bool insertToTsdf(const std::vector<cnoid::BodyPtr>& bodies, // input
                    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdfIntegrator, // in out
                    double ray, // input
                    double resolution, // input
                    cnoid::Isometry3 mapOrigin // input
                    ){
    const cnoid::Isometry3 mapOriginInv = mapOrigin.inverse();

    for(int b=0;b<bodies.size();b++){
      for(int i=0;i<bodies[b]->numLinks();i++){
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > vertices = getSurfaceVerticesAndNormals(bodies[b]->link(i), resolution /*voxel_sizeよりも多い濃度でsampleしないと穴があきgradientが不正確*/, M_PI/3); // link local
        for(int j=0;j<vertices.size();j++){
          cnoid::Vector3 p = bodies[b]->link(i)->T() * vertices[j].first; // world frame
          cnoid::Vector3 n = (bodies[b]->link(i)->R() * vertices[j].second).normalized(); // world frame

          cnoid::Vector3 origin = p + n * ray; // カメラ原点. world frame
          cnoid::Vector3 z = -n.normalized(); // カメラ姿勢. world frame
          cnoid::Vector3 x, y;
          if(cnoid::Vector3::UnitY().cross(z).norm() > 0){
            x = cnoid::Vector3::UnitY().cross(z).normalized();
            y = z.cross(x).normalized();
          }else{
            y = z.cross(cnoid::Vector3::UnitX()).normalized();
            x = y.cross(z).normalized();
          }
          cnoid::Matrix3 R; R.col(0) = x; R.col(1) = y; R.col(2) = z; // カメラ姿勢. world frame

          cnoid::Vector3 origin_local = mapOriginInv * origin; // カメラ原点. map frame
          cnoid::Matrix3 R_local = mapOriginInv.linear() * R; // カメラ姿勢. map frame

          voxblox::Transformation trans(Eigen::Quaterniond(R_local).cast<float>(), origin_local.cast<float>()); // map frame
          voxblox::Pointcloud pcl{Eigen::Vector3f(0.0,0.0,ray)};
          voxblox::Colors color{voxblox::Color(0.5,0.5,0.5)};
          tsdfIntegrator->integratePointCloud(trans, pcl, color);
        }
      }
    }

    // esdfI2ntegrator->addNewRobotPosition(voxblox::Point(0.0, 0.0, 0.0)); // clearする.
    // esdfIntegrator->updateFromTsdfLayer(true);
    // meshIntegrator->generateMesh(true, true);

    return true;
  }

};
