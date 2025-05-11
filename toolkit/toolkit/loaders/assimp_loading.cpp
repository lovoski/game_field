#include "toolkit/loaders/mesh.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace toolkit::assets {

uniform_package load_model(std::string path) {
  uniform_package package;
  package.model_path = path;
  return package;
}

}; // namespace toolkit::assets