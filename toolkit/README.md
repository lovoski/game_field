# toolkit 文档

## 首要说明

工具包本身完全采用 c++20 编写，在 msvc 上编译，通过 cmake 构建，没有使用到编译器独有的特性，支持 window，linux 平台。（macos 原生不支持 opengl 4.6，可能无法编译）。

工具包中使用右手坐标系，以正 Y 为上方向，正 Z 为前方向（不同于 unity 中的左手系，正 Y 上方向，正 Z 前方向）；内部长度单位默认为1m；渲染时摄像机朝向负 Z 方向，遵循 opengl 的规范；采用 32 位浮点数。

`toolkit` 完全 `self-contained`，可以直接复制到别的项目中，通过以下 cmake 命令链接使用：

```cmake
add_subdirectory(toolkit)
...
target_link_libraries(main PRIVATE toolkit)
```

## TODO List

- 实现基于物理的天空盒光照，并给天空盒添加一个 directional light，采用 cascaded shadow map 实现阴影
- 完善贴图和材质系统，为后续屏幕空间光追做准备
- 迁移构建 bvh 的代码
- 参考 raw-physics 实现碰撞检测和 xpbd 物理模拟

## 修复日志

这里会记录一些花了比较长时间修复的 bug，并详细记录问题出现的原因和解决方法。

### 2025.06.08 插件运行错误

在这之前的版本，如果一个插件（script）在场景中存在多个实例，当删除其中的一个，可能导致另外一个因为内存被释放掉无法正常执行，程序报错。

原因是我将插件（script）视作一个组件（component）交给 entt 管理，当一个实体（entity）被删除时，他与组件之间的连接关系会被重置，但是组件占用的内存不会直接回收，因为 entt 会把所有同类型的组件保存到一个完整的数组中以方便存取，为了方便后续创建相同类型的组件， entt 会把被删除的组件与数组最后一个组件 swap，然后直接释放最后一个组件的内存。

我为了调用每一个插件的函数，在全局下保存了这些插件的指针。在经过上面的 swap 之后，这个指针变成了野指针，无法正常使用。

解决的方法是不再保存到插件的指针，而是保存调用 `registry.view` 的回调函数，每次都通过回调函数获取所有自动注册过的插件的 `view`，然后遍历所有插件完成操作。

## 组件（component），插件（script）和材质（material）的编写

需要注意，这里的组件（component）是指ecs架构中用于存储数据的类，可以适用于提供的editor以外的框架。但是插件（script）和材质（material）分别依赖于插件系统（scripting_system）和渲染系统（defered_forward_mixed）的实现，如果需要在别的框架中复用，需要参考对应系统中的初始化、注册，会比较麻烦。

虽然这里列出了三个名词，其实后两者都属于组件（component），所以我们可以用entt提供的统一接口为一个实体（entity）挂载这三个类：

```cpp
// 获取组件/插件/材质
auto &comp = registry.get<xxx>(xxx);
// 添加组件/插件/材质
auto &comp = registry.emplace<xxx>(xxx);
// 删除组件/插件/材质
registry.remove<xxx>(xxx);
```

如果要创建自定义的组件/插件/材质，需要继承自对应的类，并通过对应的宏声明：

```cpp
#include "toolkit/system.hpp"
class custom_component : public toolkit::icomponent {};
DECLARE_COMPONENT(custom_component, custom_category)
#include "toolkit/scriptable.hpp"
class custom_script : public toolkit::scriptable {};
DECLARE_SCRIPT(custom_script, custom_category)
#include "toolkit/opengl/components/material.hpp"
class custom_material : public toolkit::opengl::material {};
DECLARE_MATERIAL(custom_material)
```

需要注意的是，由于 `DECLARE_XXX` 宏中存在通过创建静态变量初始化的操作，我们最好确保最终编译的二进制文件链接到 `DECLARE_XXX` 宏所在文件，如果对应的文件没有链接到最终的二进制文件中，无法在 editor 内通过 gui 添加这个组件/插件/材质。不过这个问题完全不影响通过entt的接口使用这些类。

## 支持文件格式

我编写 `toolkit` 的目标不是实现一个界面、功能完善的游戏引擎取代工作中 unity，unreal 的位置，而是提供一个我自己足够熟悉的平台，快速实践一些想法，或者快速可视化一些 python 处理后的数据。

为此，这个工具包必须对多种输入文件提供支持，目前支持下面的文件：

1. `.npy,.npz`: 借助 cnpy 实现
2. `.fbx,.obj`: 借助 ufbx 实现，支持 skinned mesh 和 blend shape
3. `.bvh`: 自主实现
4. `.png,.jpg,.bmp`: 借助 stb_image 实现
5. `.json`: 借助 nlohmann json 实现

除了上面这些常用的中间结果，还有下面这些格式：

5. `.scene`: 自定义场景文件，json 格式
6. `.prefab`: 自定义 prefab 文件，json 格式

这里对场景文件和 prefab 文件做出说明。场景指的是包含了一个场景中所有的实体，组件，系统参数的文件，本身就是完整的。prefab 文件保存了一个场景中一部分实体，组件的状态，并不是完整的场景中，但是可以快速融合到现有的场景中，可以用来保存一些角色，场景。

## 插件和 ecs 编写规范

由于这个工具包的架构非常简洁，**我们必须要手动维护插件的生命周期**。具体来说，我们不能直接复制一个插件（script）变量，否则会创建一个新的插件到同一个实体上，对于插件的操作必须是引用或者指针。同时，插件内部的变量如果需要被序列化，应当确保其能够被序列化反序列化为 `nlohmann::json` 格式，对于特殊的成员（opengl buffer，texture），我们应当在 `init1` 函数中确保这个插件反序列化后对象仍然是有效的，或者自己理清楚何时如何创建这些特殊成员，何时使用这些成员。

另外，**插件内部可以安全保存实体（`entt::entity`），但是不能保存任何的组件（`component`）**，任何需要组件的操作都应该在函数的内部通过 `registry.get<T>(e)` 经过 $O(1)$ 时间获得。对于插件内部直接保存的实体（`entt::entity`），不论是直接保存当前的场景，还是保存部分场景为 prefab，我都能够确保反序列化之后保存的实体变量值被映射回当前有效的值。需要额外注意的是，保存实体不能保存实体的引用或者指针（实际上`entt::entity`大小与 `uint32_t` 相同，不必在意开销）。

为了确保插件和组件能够正确通过静态反射机制实现序列化，**所有插件和组件必须分别继承自 `scriptable` 和 `icomponent`，并分别通过宏 `DECLARE_SCRIPT` 和 `DECLARE_COMPONENT` 声明**（可以参考 `toolkit/opengl/scripts/camera.hpp` 的实现）。

插件的功能类似于 unity 中的 c# 脚本，提供以下函数模板，需要注意的是，如果有一些初始化操作要做，不能在插件的构造函数中编写，此时插件内部的 `registry` 和 `entity` 都还未正确设置，初始化操作应当在 `start` 函数中执行。对于特殊成员的初始化，我们需要在 `init1` 函数中执行（这个函数仅在反序列化场景或者 prefab 中所有的实体和组件之后对于每个组件及其子类调用一次），这种特殊成员的初始化可以参考 `toolkit/opengl/components/mesh.hpp`。

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

## 编译期反射机制

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

## 编程应当遵循的规范

为了方便代码的维护和拓展，尽量注意以下几点：
1. 把不同功能的函数份文件存放不是必需的，一切以方便浏览为前提
2. 尽量不要声明过多类内的全局变量，能够在函数内部声明的变量在内部解决
