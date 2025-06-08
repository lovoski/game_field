#include "toolkit/loaders/image.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"
#include <fstream>
#include <iostream>
#include <json.hpp>

using toolkit::assets::image;
using json = nlohmann::json;

struct test0 {
  int a = 10;
  float b = 100;
  std::vector<int> v = std::vector<int>(3);
  toolkit::math::matrix4 m;
};
REFLECT(test0, a, b, v, m)

struct test1 {
  toolkit::math::vector4 m0;
  std::array<int, 4> m1;
  std::array<float, 4> m2;
  std::array<double, 4> m3;
};
REFLECT(test1, m0, m1, m2)

#include "toolkit/opengl/components/materials/all.hpp"

int main() {
  toolkit::opengl::blinn_phong_material bp_mat;
  auto results = toolkit::opengl::parse_glsl_uniforms(
      {bp_mat.vertex_shader_source, bp_mat.fragment_shader_source,
       bp_mat.geometry_shader_source});

  std::cout << ((nlohmann::json)results).dump(2) << std::endl;

  // std::string filepath;
  // if (toolkit::open_file_dialog("Select image file", {"*.png"}, "*.png",
  // filepath)) {
  //   image img;
  //   img.load(filepath);
  //   nlohmann::json data = img;
  //   std::ofstream output("test_image.json");
  //   output << data.dump(2) << std::endl;
  //   output.close();
  //   image img1 = data.get<image>();
  //   img1.save_png("test_image.png");
  // }

  // std::cout << toolkit::abspath("test_image.png") << std::endl;
  // std::cout << toolkit::relpath(
  //                  "C:\\repo\\toolkit\\build\\Debug\\test_image.png")
  //           << std::endl;

  // test1 t1;

  // nlohmann::json data = t1;

  // std::cout << data.dump(2) << std::endl;

  return 0;
}
void testImg() {
  image img, saveImg, cleanImg;
  img.load("D:\\0Task\\avatar\\cat_pixel.png");

  const int cellSize = 16;
  printf("(%d,%d)\n", img.width, img.height);

  saveImg.resize(img.width / cellSize, img.height / cellSize, 3);
  cleanImg.resize(img.width, img.height, 3);

  for (int i = 0; i < img.width / cellSize; i++) {
    for (int j = 0; j < img.height / cellSize; j++) {
      float r = 0, g = 0, b = 0;
      for (int ki = 0; ki < cellSize; ki++) {
        for (int kj = 0; kj < cellSize; kj++) {
          int x = cellSize * i + ki;
          int y = cellSize * j + kj;
          r += img.pixel(x, y, 0);
          g += img.pixel(x, y, 1);
          b += img.pixel(x, y, 2);
        }
      }
      saveImg.pixel(i, j, 0) = r / (cellSize * cellSize);
      saveImg.pixel(i, j, 1) = g / (cellSize * cellSize);
      saveImg.pixel(i, j, 2) = b / (cellSize * cellSize);
      for (int ki = 0; ki < cellSize; ki++) {
        for (int kj = 0; kj < cellSize; kj++) {
          int x = cellSize * i + ki;
          int y = cellSize * j + kj;
          cleanImg.pixel(x, y, 0) = r / (cellSize * cellSize);
          cleanImg.pixel(x, y, 1) = g / (cellSize * cellSize);
          cleanImg.pixel(x, y, 2) = b / (cellSize * cellSize);
        }
      }
    }
  }

  saveImg.save_png("D:\\0Task\\avatar\\cat_pixel_64x64.png");
  cleanImg.save_png("D:\\0Task\\avatar\\cat_pixel_1024x1024.png");
}