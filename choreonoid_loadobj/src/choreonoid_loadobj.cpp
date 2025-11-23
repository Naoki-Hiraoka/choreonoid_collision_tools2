#include <choreonoid_loadobj/choreonoid_loadobj.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <ros/package.h>
#include <cnoid/BodyLoader>
#include <iostream>
#include <fstream>

namespace choreonoid_loadobj {

  std::string parseFileName(const std::string& fileName){
    std::regex re("\\$\\(find ([^)]+)\\)");
    std::smatch match;
    if (std::regex_search(fileName, match, re)) {
      return ros::package::getPath(match[1]) + std::string(match.suffix().str());
    } else {
      return fileName;
    }
  }

  bool loadObjectsFromYAML(const std::string& yamlFileName, std::vector<cnoid::BodyPtr>& bodies){
    std::ifstream ifs(yamlFileName);
    if(!ifs.is_open()){
      std::cerr << __FUNCTION__ << "file not found: " << yamlFileName << std::endl;
      return false;
    }
    bodies.clear();
    cnoid::BodyLoader bodyLoader;
    YAML::Node param = YAML::Load(ifs);
    if(!param.IsMap()) return false;
    for(YAML::const_iterator it=param.begin();it!=param.end();it++) {
      std::string objName = it->first.as<std::string>();
      YAML::Node objInfo = it->second;
      {
        YAML::Node node = objInfo["name"];
        if(node) objName = node.as<std::string>();
      }
      std::string fileName;
      {
        YAML::Node node = objInfo["file"];
        if(!node){
          std::cerr << __FUNCTION__ << "file not found in yaml!" << std::endl;
          continue;
        }
        fileName = parseFileName(node.as<std::string>());
      }
      cnoid::BodyPtr body = bodyLoader.load(fileName);
      if(!body){
        std::cerr << __FUNCTION__ << "file not found: " << fileName << std::endl;
        continue;
      }
      body->setName(objName);
      {
        YAML::Node node = objInfo["translation"];
        if(node){
          for(int i=0;i<3;i++) body->rootLink()->p()[i] = node[i].as<double>();
        }
      }
      {
        YAML::Node node = objInfo["rotation"];
        if(node){
          for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
              body->rootLink()->R()(i,j) = node[i][j].as<double>();
            }
          }
        }
      }
      {
        YAML::Node node = objInfo["static"];
        if(node){
          if(node.as<bool>()){
            body->rootLink()->setJointType(cnoid::Link::JointType::FixedJoint);
            body->updateLinkTree();
          }
        }
      }
      {
        YAML::Node node = objInfo["static_joint"];
        if(node){
          if(node.as<bool>()){
            for(int j=0;j<body->numAllJoints();j++){
              body->joint(j)->setJointType(cnoid::Link::JointType::FixedJoint);
            }
            body->updateLinkTree();
          }
        }
      }
      {
        YAML::Node node = objInfo["static_joints"];
        if(node){
          for(int j=0;j<node.size();j++){
            std::string jointName = node[j].as<std::string>();
            cnoid::LinkPtr link = body->link(jointName);
            if(!link){
              std::cerr << __FUNCTION__ << "link not found: " << jointName << std::endl;
              continue;
            }
            link->setJointType(cnoid::Link::JointType::FixedJoint);
          }
          body->updateLinkTree();
        }
      }
      body->calcForwardKinematics();
      bodies.push_back(body);
    }
    return true;
  }
};
