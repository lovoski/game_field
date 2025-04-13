#include "ToolKit/Loaders/Image.hpp"
#include "ToolKit/Reflect.hpp"
#include "ToolKit/System.hpp"
#include "ToolKit/Transform.hpp"
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

int main() {
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