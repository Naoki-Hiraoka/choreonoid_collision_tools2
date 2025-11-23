#include <choreonoid_voxblox/choreonoid_voxblox.h>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>
#include <cnoid/MeshGenerator>
#include <choreonoid_viewer/choreonoid_viewer.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/planning_utils.h>
#include <voxblox/io/layer_io.h>


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
    model->updateBoundingBox();
    return model;
  }
  cnoid::BodyPtr convertToChoreonoidBody(const std::shared_ptr<voxblox::TsdfMap>& tsdf_map){

    std::shared_ptr<voxblox::MeshLayer> mesh_layer = std::make_shared<voxblox::MeshLayer>(tsdf_map->block_size());
    voxblox::MeshIntegratorConfig mesh_config;
    std::shared_ptr<voxblox::MeshIntegrator<voxblox::TsdfVoxel> > meshIntegrator = std::make_shared<voxblox::MeshIntegrator<voxblox::TsdfVoxel> >(mesh_config, tsdf_map->getTsdfLayerPtr(), mesh_layer.get());
    meshIntegrator->generateMesh(true, true);

    cnoid::BodyPtr meshBody = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          std::shared_ptr<voxblox::Mesh> mesh = std::make_shared<voxblox::Mesh>();
          mesh_layer->getMesh(mesh.get());
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(choreonoid_voxblox::convertToChoreonoidModel(mesh));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6)); // meshのcolorは鏡面反射以外の要素が弱く、暗くなる.
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,0);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
        }
        meshBody->setRootLink(rootLink);
      }
    }
    return meshBody;
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
          voxblox::Colors color{voxblox::Color(120,120,120)};
          tsdfIntegrator->integratePointCloud(trans, pcl, color);
        }
      }
    }

    // esdfI2ntegrator->addNewRobotPosition(voxblox::Point(0.0, 0.0, 0.0)); // clearする.
    // esdfIntegrator->updateFromTsdfLayer(true);
    // meshIntegrator->generateMesh(true, true);

    return true;
  }

  bool insertToTsdf(const cnoid::RangeCameraPtr camera, // input
                    std::shared_ptr<voxblox::TsdfIntegratorBase> tsdfIntegrator, // in out
                    cnoid::Isometry3 mapOrigin // input
                    ){
    const cnoid::Isometry3 mapOriginInv = mapOrigin.inverse(); // world frame
    const cnoid::Isometry3 cameraPose = mapOriginInv * camera->link()->T() * camera->T_local(); // map frame

    const std::vector<cnoid::Vector3f>& points = camera->constPoints(); // camera frame
    const unsigned char* pixels = camera->constImage().pixels();

    voxblox::Transformation trans(Eigen::Quaterniond(cameraPose.linear()).cast<float>(), cameraPose.translation().cast<float>()); // map frame
    voxblox::Pointcloud pcl;
    voxblox::Colors color;

    pcl.reserve(points.size());
    color.reserve(points.size());
    for(int i=0;i<points.size();i++){
      if(!points[i].allFinite()){
        pixels+=3;
        continue;
      }
      pcl.push_back(points[i]);
      if (camera->imageType() == cnoid::Camera::COLOR_IMAGE) {
        unsigned char r = *pixels++;
        unsigned char g = *pixels++;
        unsigned char b = *pixels++;
        color.push_back(voxblox::Color(r,g,b));
      }else{
        color.push_back(voxblox::Color(120,120,120));
      }
    }

    tsdfIntegrator->integratePointCloud(trans, pcl, color);

    // esdfI2ntegrator->addNewRobotPosition(voxblox::Point(0.0, 0.0, 0.0)); // clearする.
    // esdfIntegrator->updateFromTsdfLayer(true);
    // meshIntegrator->generateMesh(true, true);

    return true;
  }

  bool calcEsdf(const std::vector<cnoid::BodyPtr>& obstacles, // input
                std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> >& collisionModels, // input
                std::shared_ptr<voxblox::TsdfMap>& tsdf_map, // output
                std::shared_ptr<voxblox::EsdfMap>& esdf_map, // output
                const calcEsdfParam& param
                ){
    voxblox::TsdfMap::Config tsdf_config;
    tsdf_config.tsdf_voxel_size = param.voxel_size;
    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel> > tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel> >(tsdf_config.tsdf_voxel_size, tsdf_config.tsdf_voxels_per_side);
    tsdf_map = std::make_shared<voxblox::TsdfMap>(tsdf_layer);

    voxblox::EsdfMap::Config esdf_config;
    esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
    std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel> > esdf_layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel> >(esdf_config.esdf_voxel_size, esdf_config.esdf_voxels_per_side);
    esdf_map = std::make_shared<voxblox::EsdfMap>(esdf_layer);

    voxblox::TsdfIntegratorBase::Config tsdf_integrator_config;
    tsdf_integrator_config.voxel_carving_enabled = true;
    tsdf_integrator_config.default_truncation_distance = tsdf_config.tsdf_voxel_size; // 環境にこの値以上めりこんだ位置は未観測扱いになるので注意
    tsdf_integrator_config.max_weight = 10000.0; // default=10000. 30程度だと面が凸凹になる.
    tsdf_integrator_config.min_ray_length_m = 0.01;
    tsdf_integrator_config.use_sparsity_compensation_factor = true; // static model限定. 細いものが消えないように
    tsdf_integrator_config.sparsity_compensation_factor = 1000.0;  // static model限定. 非常に大きな値. 消えなくなるのでdynamic modelで用いるべきではない
    std::shared_ptr<voxblox::FastTsdfIntegrator> tsdfIntegrator = std::make_shared<voxblox::FastTsdfIntegrator>(tsdf_integrator_config, tsdf_layer.get());

    voxblox::EsdfIntegrator::Config esdf_integrator_config;
    esdf_integrator_config.min_distance_m = param.voxel_size;
    esdf_integrator_config.max_distance_m = param.default_distance;
    esdf_integrator_config.default_distance_m = param.default_distance;
    esdf_integrator_config.clear_sphere_radius = 1.0;
    //esdf_integrator_config.full_euclidean_distance = true;
    std::shared_ptr<voxblox::EsdfIntegrator> esdfIntegrator = std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config, tsdf_map->getTsdfLayerPtr(), esdf_layer.get());
    // esdfIntegrator->addNewRobotPosition(center); // centerを中心としたclear_sphere_radiusの球をobservedかつdefault_distance_mにセットする. esdf_mapはobservedでないvoxelに対して距離を取得しようとすると失敗してfalseを返すが、rayが通っていないvoxelはobservedでないため困る.

    std::vector<cnoid::Matrix3> rotations{cnoid::Matrix3(cnoid::AngleAxisd(0,cnoid::Vector3::UnitZ())),
                                          cnoid::Matrix3(cnoid::AngleAxisd(M_PI/2,cnoid::Vector3::UnitZ())),
                                          cnoid::Matrix3(cnoid::AngleAxisd(M_PI,cnoid::Vector3::UnitZ())),
                                          cnoid::Matrix3(cnoid::AngleAxisd(-M_PI/2,cnoid::Vector3::UnitZ())),
                                          cnoid::Matrix3(cnoid::AngleAxisd(M_PI/2,cnoid::Vector3::UnitY())),
                                          cnoid::Matrix3(cnoid::AngleAxisd(-M_PI/2,cnoid::Vector3::UnitY()))};


    cnoid::BodyPtr cameraBody = new cnoid::Body();
    {
      cnoid::MeshGenerator meshGenerator;
      {
        cnoid::LinkPtr rootLink = new cnoid::Link();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.05,0.05,0.05)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          rootLink->addVisualShapeNode(shape);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.1,0.1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.6, 0.6, 0.6));
          shape->setMaterial(material);
          rootLink->addCollisionShapeNode(shape);
        }
        cameraBody->setRootLink(rootLink);
      }
      {
        cnoid::RangeCameraPtr camera = new cnoid::RangeCamera();
        camera->setName("camera");
        camera->T_local().translation() << 0.0, 0.0, 0.0;
        camera->T_local().linear() = cnoid::Matrix3(cnoid::AngleAxisd(-M_PI/2,cnoid::Vector3::UnitZ())*cnoid::AngleAxisd(M_PI/2,cnoid::Vector3::UnitX()));
        camera->setFrameRate(1000);
        camera->setFarClipDistance(200.0);
        camera->setNearClipDistance(0.04);
        camera->setFieldOfView(M_PI / 2);
        camera->setResolution(param.resolution,param.resolution);
        camera->setImageType(cnoid::Camera::COLOR_IMAGE);
        camera->setMaxDistance(param.maxDistance);
        camera->setMinDistance(0.04);
        camera->setOrganized(true);
        cameraBody->addDevice(camera, cameraBody->rootLink());
      }
    }
    cameraBody->calcForwardKinematics();

    cnoid::RangeCameraPtr camera = cameraBody->findDevice<cnoid::RangeCamera>("camera");
    cnoid::LinkPtr cameraLink = cameraBody->rootLink();
    std::shared_ptr<btConvexShape> cameraCollisionModel = choreonoid_bullet::convertToBulletModel(cameraLink->collisionShape());

    for(int b=0;b<obstacles.size();b++){
      for(int l=0;l<obstacles[b]->numLinks();l++){
        if(collisionModels.find(obstacles[b]->link(l)) == collisionModels.end()){
          collisionModels[obstacles[b]->link(l)] = choreonoid_bullet::convertToBulletModel(obstacles[b]->link(l)->collisionShape());
        }
      }
    }

    std::unique_ptr<choreonoid_viewer::Viewer> viewer = std::make_unique<choreonoid_viewer::Viewer>();
    viewer->timeStep = 0.001;
    viewer->objects(obstacles);
    viewer->objects(cameraBody);
    viewer->cameras(camera);
    viewer->drawObjects(true);

    std::cerr << "generating TSDF" << std::endl;

    for(double z = param.min_z; z <= param.max_z; z+=param.step){ // voxel sizeよりも薄い物体を両側から観測すると、消滅する恐れがある. 一番重要なのは水平面なので、z軸を一番外側のループにする
      for(double x = param.min_x; x <= param.max_x; x+=param.step){
        for(double y = param.min_y; y <= param.max_y; y+=param.step){
          cameraBody->rootLink()->p() << x, y, z;
          cameraBody->calcForwardKinematics();
          bool noCollision = true;
          for(int b=0;b<obstacles.size();b++){
            for(int l=0;l<obstacles[b]->numLinks();l++){
              cnoid::LinkPtr link = obstacles[b]->link(l);
              std::shared_ptr<btConvexShape> btShape = collisionModels[link];

              cnoid::Vector3 A_localp, B_localp;
              double dist;
              bool solved = choreonoid_bullet::computeDistance(cameraCollisionModel,
                                                               cameraLink->p(),
                                                               cameraLink->R(),
                                                               btShape,
                                                               link->p(),
                                                               link->R(),
                                                               dist,
                                                               A_localp,
                                                               B_localp
                                                               );
              if(solved && dist < 0.0) noCollision = false; // cameraBodyの形状が上下左右前後対称である仮定
            }
          }
          if(!noCollision) continue;
          for(int r=0;r<rotations.size();r++){
            cameraBody->rootLink()->R() = rotations[r];
            cameraBody->calcForwardKinematics();
            viewer->drawObjects(true);
            choreonoid_voxblox::insertToTsdf(camera, tsdfIntegrator, cnoid::Isometry3::Identity());
          }
        }
      }
    }

    std::cerr << "generating ESDF" << std::endl;

    //esdfIntegrator->updateFromTsdfLayer(true);
    esdfIntegrator->updateFromTsdfLayerBatch();

    std::cerr << "generated" << std::endl;

    return true;
  }

  bool loadOrCalcEsdf(const std::string& vxblxFileName,
                      const std::vector<cnoid::BodyPtr>& obstacles, // input
                      std::unordered_map<cnoid::LinkPtr, std::shared_ptr<btConvexShape> >& collisionModels, // input
                      std::shared_ptr<voxblox::TsdfMap>& tsdf_map, // output
                      std::shared_ptr<voxblox::EsdfMap>& esdf_map, // output
                      const calcEsdfParam& param
                      ){
    bool success = false;

    {
      voxblox::TsdfMap::Config tsdf_config;
      tsdf_config.tsdf_voxel_size = param.voxel_size;
      std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel> > tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel> >(tsdf_config.tsdf_voxel_size, tsdf_config.tsdf_voxels_per_side);
      tsdf_map = std::make_shared<voxblox::TsdfMap>(tsdf_layer);

      voxblox::EsdfMap::Config esdf_config;
      esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
      std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel> > esdf_layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel> >(esdf_config.esdf_voxel_size, esdf_config.esdf_voxels_per_side);
      esdf_map = std::make_shared<voxblox::EsdfMap>(esdf_layer);

      success =
        voxblox::io::LoadBlocksFromFile(vxblxFileName,
                                        voxblox::Layer<voxblox::TsdfVoxel>::BlockMergingStrategy::kReplace,
                                        true,
                                        tsdf_layer.get())
        &&
        voxblox::io::LoadBlocksFromFile(vxblxFileName,
                                        voxblox::Layer<voxblox::EsdfVoxel>::BlockMergingStrategy::kReplace,
                                        true,
                                        esdf_layer.get());
    }

    if(success) return true;

    bool result = calcEsdf(obstacles,
                           collisionModels,
                           tsdf_map,
                           esdf_map,
                           param);
    if(!result) return false;

    voxblox::io::SaveLayer(*(tsdf_map->getTsdfLayerPtr()),
                           vxblxFileName);
    voxblox::io::SaveLayer(*(esdf_map->getEsdfLayerPtr()),
                           vxblxFileName,
                           false);
    return true;
  }


};
