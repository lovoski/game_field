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
6. `.bundle`: 自定义 bundle 文件，json 格式

这里对场景文件和 bundle 文件做出说明。场景指的是包含了一个场景中所有的实体，组件，系统参数的文件，本身就是完整的。bundle 文件保存了一个场景中一部分实体，组件的状态，并不是完整的场景中，但是可以快速融合到现有的场景中，可以用来保存一些角色，场景。

## script 的说明

script 的概念非常类似 unity 中的 c# script。在使用方式上两者基本保持一致，不过本引擎中的 script 其实是引擎源代码的一部分，需要直接编译到引擎中发生效果，也不支持热更新。

所有的 script 都应当继承自 scriptable 基类。虽然 c++ 带有构造函数和析构函数，一个 script 的成员变量初始化和销毁应该在重载的 `start` 和 `destroy` 函数中进行。每一个 script 都会带有两个默认的变量 `registry` 以及 `entity`，系统确保这两个变量在 start 之前一定是有效的。其中 `registry` 是当前场景的 ecs registry，可以用于访问和管理场景中的所有 entity，component 和 system；`entity` 则是挂载了当前 script 的实体，通过 `entity` 我们可以管理当前实体上挂载的其他 component。

所有 script 都需要在末尾通过宏 `DECLARE_SCRIPT` 声明，由于已经引擎中包含简单的编译时期反射，可以直接将需要序列化的数据声明到 `DECLARE_SCRIPT` 中。script 的成员变量中可以保存 entity 以及 entity 的模板，但是不应该保存 component 的拷贝，指针或者引用，对于 component 的可持久化非常容易因为内存管理无效，对于 component 的管理应该直接在运行时实时获取。

如果需要在 script 中创建新的 entity 并保存，应当在 `start` 函数中确保实体被正确初始化，具体可以参考 `scripts/mixamo_manipulate.hpp`。

引擎采用了非常简易的 main loop，所有函数单线程执行，其中 `preupdate`, `update` 和 `lateupdate` 均与 unity 中的用法相同。

## 编程应当遵循的规范

为了方便代码的维护和拓展，尽量注意以下几点：
1. 把不同功能的函数份文件存放不是必需的，一切以方便浏览为前提
2. 尽量不要声明过多类内的全局变量，能够在函数内部声明的变量在内部解决
