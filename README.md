# 说明文档

本项目主体包括下面的几个部分：

1. `toolkit`: ecs 架构的精简游戏引擎框架，是其他项目的基础，同时也包含一个 editor 提供可视化编辑，editor 通过 opengl 4.3 可视化，实现了简易但是易于使用的编译期反射，包含序列化和 c++ 插件功能
2. `manga_viewer`: 可以读入 **.pdf**,**.epub** 格式电子书的阅读器，基于 `toolkit` 封装的 opengl 命令，实现了仿真翻页功能，通过异步加载尽可能避免了加载电子书的时间
3. `python`: 用于数据处理的 python 代码，包含了读入 bvh 文件的库，可能用到的 motion builder 和 blender 脚本
5. `scripts`: 适用于 toolkit 中 editor 的 c++ 插件（一般用来可视化数据）

## 功能介绍

### 首要说明

工具包本身完全采用 c++20 编写，在 msvc 上编译，通过 cmake 构建，没有使用到编译器独有的特性，支持 window，linux，macos 平台。（macos 原生不支持 opengl 4.3，可能无法编译）。

工具包中使用右手坐标系，以正 Y 为上方向，正 Z 为前方向（不同于 unity 中的左手系，正 Y 上方向，正 Z 前方向）。渲染时摄像机朝向负 Z 方向，遵循 opengl 的规范。采用 32 位浮点数。

`toolkit` 完全 `self-contained`，可以直接复制到别的项目中，通过以下 cmake 命令链接使用：

```cmake
add_subdirectory(toolkit)
...
target_link_libraries(main PRIVATE toolkit)
```

### 支持文件格式

我编写 `toolkit` 的目标不是实现一个界面、功能完善的游戏引擎取代工作中 unity，unreal 的位置，而是提供一个我自己足够熟悉的平台，快速实践一些想法，或者快速可视化一些 python 处理后的数据。

为此，这个工具包必须对多种输入文件提供支持，目前支持下面的文件：

1. `.npy,.npz`: 借助 cnpy 实现
2. `.fbx,.obj`: 借助 assimp 实现，支持 skinned mesh 和 blend shape
3. `.bvh`: 自主实现
4. `.png,.jpg,.bmp`: 借助 stb_image 实现
5. `.json`: 借助 nlohmann json 实现

除了上面这些常用的中间结果，还有下面这些格式：

5. `.scene`: 自定义场景文件，json 格式
6. `.prefab`: 自定义 prefab 文件，json 格式

### 插件和 ecs 编写规范

为了确保插件和组件能够正确通过静态反射机制实现序列化，所有插件和组件必须分别继承自 `scriptable` 和 `icomponent`，并分别通过宏 `DECLARE_SCRIPT` 和 `DECLARE_COMPONENT` 声明：

```cpp
// Specifications for a script (插件)
class editor_camera : public scriptable {
public:
  bool mouse_first_move = true;
  math::vector2 mouse_last_pos;
  math::vector3 camera_pivot{0.0, 0.0, 0.0};

  // Some parameter related to camera control
  float initial_factor = 0.6f;
  float speed_pow = 1.5f;
  float max_speed = 8e2f;
  float fps_speed = 1.2;

  bool vis_pivot = false;
  float vis_pivot_size = 0.1f;

  void preupdate(iapp *app, float dt) override;
  void draw_to_scene(iapp *app) override;
  void draw_gui(iapp *app) override;
};
DECLARE_SCRIPT(editor_camera, basic, mouse_first_move， mouse_last_pos,camera_pivot, initial_factor, speed_pow, max_speed, fps_speed, vis_pivot, vis_pivot_size)

// Specifications for a component (组件)
struct point_light : public icomponent {
  math::vector3 color = White;

  bool enabled = true;
};
DECLARE_COMPONENT(point_light, graphics, color, enabled)
```

插件的功能类似于 unity 中的 c# 脚本，提供以下函数模板：

```cpp
// toolkit/scriptable.hpp
class scriptable : public icomponent {
public:
  scriptable() { __registered_scripts__.insert(this); }

  virtual void start() {}
  virtual void destroy() {}

  virtual void draw_to_scene(iapp *app) {}

  virtual void preupdate(iapp *app, float dt) {}
  virtual void update(iapp *app, float dt) {}
  virtual void lateupdate(iapp *app, float dt) {}

  bool enabled = true;

  entt::registry *registry = nullptr;
  entt::entity entity = entt::null;

  static inline std::set<scriptable *> __registered_scripts__;
};
```

组件则是服务于 entt 实现的 ecs 结构，一般用来单纯保存数据，为了避免编写过于复杂，提供了下面的函数模板：

```cpp
// toolkit/system.hpp
class icomponent {
public:
  virtual void init1() {}
  virtual void draw_gui(iapp *app) {}
  virtual std::string get_name() { return typeid(*this).name(); }
};
```

### 编译期反射机制

静态反射机制通过 c++17 的 tuple 结合 `FOR_EACH` 宏实现，主要逻辑位于 `toolkit/reflect.hpp`。在这个宏的基础上 `DECLARE_SCRIPT` 和 `DECLARE_COMPONENT` 注册了对应 script 和 component 的功能函数。

所有添加了反射宏 `REFLECT` 的结构均会获得序列化反序列化功能，用法如下：

```cpp
struct test0 {
  toolkit::math::vector4 m0;
  std::array<int, 4> m1;
  std::array<float, 4> m2;
  std::array<double, 4> m3;
};
REFLECT(test0, m0, m1, m2)
```

除了基本数据类型，stl 模板类型，eigen 矩阵和四元数，如果希望序列化结构中不属于这些类型的成员，只需要确保这个成员也定义了反射宏，因此以下的结构是合法的，可以正确反射：

```cpp
struct test1 {
  int a = 10;
  float b = 100;
  std::vector<int> v = std::vector<int>(3);
  toolkit::math::matrix4 m;

  test0 t0; // this is legal since `test0` is reflected.
};
REFLECT(test1, a, b, v, m, t1)
```
