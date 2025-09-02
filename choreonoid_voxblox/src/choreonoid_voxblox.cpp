#include <choreonoid_voxblox/choreonoid_voxblox.h>

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
};
