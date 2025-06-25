# 说明文档

本项目主体包括下面的几个部分：

1. [toolkit](./toolkit/README.md): ecs 架构的精简游戏引擎框架，是其他项目的基础，需要指出这是一个游戏引擎框架而不是单纯的渲染框架。这里面也包含一个 editor 能够对场景提供可视化编辑，可以增改一个实体（entity）对应的组件（component），调整各个系统的设置并序列化到文件。场景和prefab的序列化是通过静态反射实现的；在这个框架中增加组件，材质，插件步骤非常简单，框架中本身也有很多例子。
2. [manga_viewer](./manga_viewer/README.md): 可以读入 **.pdf**,**.epub** 格式电子书的阅读器，利用了 `toolkit` 封装的 opengl 命令，通过 compute shader 实现了仿真翻页功能，对于文件的读入采用了 `mupdf` 库，编译出来的二进制文件有点大。
3. `python`: 用于数据处理的 python 代码。包含了读入 bvh 文件，对于动作数据处理的库；可能用到的 motion builder 和 blender 脚本。
5. `scripts`: 适用于 toolkit 中 editor 的 c++ 插件（一般用来可视化数据）。
