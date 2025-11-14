# Document

## Important

工具包本身完全采用 c++20 编写，在 msvc 和 gcc 上通过了编译，支持 window，linux 平台。

类似 unreal engine，工具包中使用右手坐标系（正 Y 为上方向，正 Z 为前方向）；内部长度单位默认为1m；渲染时摄像机朝向负 Z 方向，遵循 opengl 的规范；采用 32 位浮点数；数学库直接采用 Eigen。

`toolkit` 可以直接复制到别的项目中，通过以下 cmake 命令链接使用：

```cmake
add_subdirectory(toolkit)
...
target_link_libraries(main PRIVATE toolkit)
```