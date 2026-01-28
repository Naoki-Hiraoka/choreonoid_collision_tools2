#ifndef CHOREONOID_LOADOBJ_CHOREONOID_LOADOBJ_H
#define CHOREONOID_LOADOBJ_CHOREONOID_LOADOBJ_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <cnoid/RangeCamera>

namespace choreonoid_loadobj {
  bool loadObjectsFromYAML(const std::string& yamlFileName, std::vector<cnoid::BodyPtr>& bodies);
};

#endif
