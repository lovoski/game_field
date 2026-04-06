# toolkit library

This is my personal toolkit dedicated to computer graphics algorithm implementation.

## How to compile

This code base requires c++20 compatible compiler. I use vcpkg for most of the dependency management.

```cmd
vcpkg install assimp glad zlib cgal sdl2 sdl2-image sdl2-ttf sdl2-mixer
```

The default font can be found [here](https://drive.google.com/file/d/17pHVEJhTXmY_WpZ4PlKX7iFsDCJjwWYT/view?usp=sharing).

## 编写原则

每一个应用算法的实现，都应该在最为纯净可控的环境中，例如 motion matching，trajectory follow，camdm 等算法的实现，都应该作为完全独立的应用，以获得最容易，最直接，效果最好的展示结果。而渲染系统，3d 物理系统，层级 transform 系统应该作为可以方便接入的模块，用于在实现具体应用算法时能够轻松调用。额外的模块，例如 smpl，motion retargeting 应该作为独立的函数，能够在需要的应用中被调用，其余时候可以被直接忽略。

为此应用的框架应当被限定在最小，以提供给使用者最大程度的自由，对于复杂框架的维护在之后也会非常困难，每一个应用在编写成型之后自成一体，互不相干。由于目前系统需要的主要模块（层级结构，渲染，物理模拟）都已经基本确定，所有的 ui 都可以通过确定硬编码的方式给定，不需要强调过多的可拓展性，额外的模块应当通过修改硬编码自行实现，避免框架过于臃肿。

维持 DECLARE_COMPONENT，DECLARE_SYSTEM 和继承范式的主要原因是为了支持基于反射的序列化和反序列化，除了统一的序列化反序列化以外不应该对于这些系统施加额外多余的规范化要求（例如统一的 update，draw_gui 等等），一切从简。

对于我不够擅长领域的代码（物理模拟，模型加载，真实感渲染），直接复用质量较高的相关代码，虽然会导致代码风格不那么统一，但是一切以实现需求为先。
