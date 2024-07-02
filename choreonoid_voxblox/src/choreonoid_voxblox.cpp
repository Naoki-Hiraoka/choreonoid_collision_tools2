#include <choreonoid_voxblox/choreonoid_voxblox.h>

namespace choreonoid_voxblox {
  // collisionshape全体で1つのmeshにする
  cnoid::SgMeshPtr convertToChoreonoidModel(const std::shared_ptr<const voxblox::Mesh>& mesh) {
    cnoid::SgMeshPtr model = new cnoid::SgMesh;
    model->getOrCreateVertices();
    model->getOrCreateNormals();
    for(int i=0;i<=mesh->vertices.size();i++){
      model->vertices()->push_back(mesh->vertices[i]);
      model->normals()->push_back(mesh->normals[i]);
    }
    for(int i=0; i+2 <mesh->indices.size(); i+=3){
      model->addTriangle(mesh->indices[i],
                         mesh->indices[i+1],
                         mesh->indices[i+2]);
      model->normalIndices().push_back(mesh->indices[i]);
      model->normalIndices().push_back(mesh->indices[i+1]);
      model->normalIndices().push_back(mesh->indices[i+2]);
    }
    return model;
  }
};
