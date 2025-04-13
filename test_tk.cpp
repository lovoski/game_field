#include "ToolKit/Loaders/Image.hpp"
#include "ToolKit/Reflect.hpp"
#include "ToolKit/System.hpp"
#include "ToolKit/Transform.hpp"
#include <fstream>
#include <iostream>
#include <json.hpp>

using toolkit::assets::image;
using json = nlohmann::json;

class Renderable {
public:
  toolkit::math::vector3 diffuse, specular, roughness;

  DECLARE_COMPONENT(Renderable, graphcis, diffuse, specular, roughness)
};

class Test1 {
public:
  Test1(int _a = 0) : a(_a) {}

  int a = 0, b = 0, c = 0;
  toolkit::math::quat q;

  toolkit::math::matrix4 m4 = toolkit::math::matrix4::Identity();
  Eigen::MatrixXd mx;

  REFLECT(Test1, a, b, c, q, m4, mx)
};

class Test2 {
public:
  Test1 t1;
  toolkit::math::vector3 vec;
  float a = 10;
  std::vector<Test1> t1s;

  REFLECT(Test2, t1, a, t1s)
};

class Test3 : public Test1 {
public:
  float t2;

  REFLECT(Test3, t2)
};

class Test4 {
public:
  Test4(int a) {}
};

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
class test_script0 : public toolkit::scriptable<test_script0> {
public:
  int a = 10;

  void update(toolkit::iapp *app, float dt) {
    std::cout << "update test_script0" << std::endl;
  }

  DECLARE_SCRIPT(test_script0, a)
};
class test_script1 : public toolkit::scriptable<test_script1> {
public:
  int b = 10;

  void update(toolkit::iapp *app, float dt) {
    std::cout << "update test_script1" << std::endl;
    if (auto ptr = dynamic_cast<toolkit::opengl::editor *>(app)) {
      std::cout << "app is editor in test_script1" << std::endl;
    }
  }

  DECLARE_SCRIPT(test_script1, b)
};

int main() {
  // test_script0 ts00, ts01;
  // test_script1 ts10, ts11;

  toolkit::opengl::editor editor;

  // for (auto script : toolkit::opengl::editor_script::scripts) {
  //   script->update(editor, 0);
  //   std::cout << script->serialize().dump() << std::endl;
  // }

  static_assert(toolkit::reflected<Test1>::value, "Test1 reflected");
  Test1 t;
  Test2 t2;
  if (!toolkit::is_reflectable<Test1>()) {
    printf("test1 not reflectable\n");
  } else {
    printf("test1 reflectable\n");
  }
  if (!toolkit::is_reflectable<Test2>()) {
    printf("test2 not reflectable\n");
  } else {
    printf("test2 reflectable\n");
  }
  auto fields = t.get_reflect_fields();
  toolkit::for_each(fields, [&](auto p) { std::cout << p.name << std::endl; });

  t.for_each_reflect_field([&](auto p) {
    std::cout << p.name << "," << typeid(p.ptr).name() << "," << (t.*(p.ptr))
              << std::endl;
  });

  std::cout << t.get_reflect_name() << std::endl;

  Test3 t3;
  std::cout << "t3:" << t3.serialize().dump() << std::endl;

  auto data = t.serialize();
  std::cout << data.dump() << std::endl;

  t.deserialize(data);
  t.q = toolkit::math::quat::Identity();
  data = t;
  std::cout << data.dump() << std::endl;
  t = data.get<Test1>();

  t2.t1.deserialize(data);
  std::cout << t2.serialize().dump() << std::endl;

  std::vector<Test1> t1s(10);
  data = t1s;
  std::cout << data.dump() << std::endl;

  toolkit::iapp app;
  auto transformSystem = app.add_sys<toolkit::transform_system>();
  auto es_sys = app.add_sys<toolkit::script_system>();

  auto ent = app.registry.create();
  auto &trans = app.registry.emplace<toolkit::transform>(ent);
  auto &s0 = app.registry.emplace<test_script0>(ent);
  auto &s1 = app.registry.emplace<test_script1>(ent);
  trans.name = "new transform";
  auto &render = app.registry.emplace<Renderable>(ent);
  render.diffuse << 0.1, 0.5, 0.9;
  // std::cout << trans.serialize().dump() << std::endl;
  for (int i = 0; i < 100; i++)
    auto ent = app.registry.create();

  app.update(0);
  es_sys->update(&editor, 0);

  auto data000 = app.serialize();
  // std::cout << data000.dump(2) << std::endl;

  app.deserialize(data000);

  es_sys = app.add_sys<toolkit::script_system>();
  es_sys->update(&editor, 0);
  app.update(0);

  // app.registry.view<Renderable>().each(
  //     [&](entt::entity entity, Renderable &render) {
  //       std::cout << render.serialize().dump(2) << std::endl;
  //     });

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