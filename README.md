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

这里对场景文件和 prefab 文件做出说明。场景指的是包含了一个场景中所有的实体，组件，系统参数的文件，本身就是完整的。prefab 文件保存了一个场景中一部分实体，组件的状态，并不是完整的场景中，但是可以快速融合到现有的场景中，可以用来保存一些角色，场景。

### 插件和 ecs 编写规范

由于这个工具包的架构非常简洁，**我们必须要手动维护插件的生命周期**。具体来说，我们不能直接复制一个插件（script）变量，否则会创建一个新的插件到同一个实体上，对于插件的操作必须是引用或者指针。同时，插件内部的变量如果需要被序列化，应当确保其能够被序列化反序列化为 `nlohmann::json` 格式，对于特殊的成员（opengl buffer，texture），我们应当在 `init1` 函数中确保这个插件反序列化后对象仍然是有效的，或者自己理清楚何时如何创建这些特殊成员，何时使用这些成员。

另外，**插件内部可以安全保存实体（`entt::entity`），但是不能保存任何的组件（`component`）**，任何需要组件的操作都应该在函数的内部通过 `registry.get<T>(e)` 经过 $O(1)$ 时间获得。对于插件内部直接保存的实体（`entt::entity`），不论是直接保存当前的场景，还是保存部分场景为 prefab，我都能够确保反序列化之后保存的实体变量值被映射回当前有效的值。需要额外注意的是，保存实体不能保存实体的引用或者指针（实际上`entt::entity`大小与 `uint32_t` 相同，不必在意开销）。

为了确保插件和组件能够正确通过静态反射机制实现序列化，**所有插件和组件必须分别继承自 `scriptable` 和 `icomponent`，并分别通过宏 `DECLARE_SCRIPT` 和 `DECLARE_COMPONENT` 声明**：

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

插件的功能类似于 unity 中的 c# 脚本，提供以下函数模板，需要注意的是，如果有一些初始化操作要做，不能在插件的构造函数中编写，此时插件内部的 `registry` 和 `entity` 都还未正确设置，初始化操作应当在 `start` 函数中执行。对于特殊成员的初始化，我们需要在 `init1` 函数中执行（这个函数仅在反序列化场景或者 prefab 中所有的实体和组件之后对于每个组件及其子类调用一次），这种特殊成员的初始化可以参考 `toolkit/opengl/components/mesh.hpp`。

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
REFLECT(test1, a, b, v, m, t0)
```

对于反射的实现大致可以理解成通过宏函数将需要反射的成员指针存放到一个 tuple 中，随后在 `nlohmann::json` 的 `to_json` 和 `from_json` 中遍历这个 tuple，实现序列化和反序列化。

因此，其实只要能够正确转化成 `nlohmann::json` 格式的结构，就能够直接放入到反射宏中。

### 编程应当遵循的规范

为了方便代码的维护和拓展，尽量注意以下几点：
1. 把不同功能的函数份文件存放不是必需的，一切以方便浏览为前提
2. 尽量不要声明过多类内的全局变量，能够在函数内部声明的变量在内部解决
