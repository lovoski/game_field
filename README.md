# toolkit library

This is my personal toolkit dedicated to computer graphics algorithm implementation.

## How to compile

This code base requires c++20 compatible compiler. I use vcpkg for most of the dependency management. After cloneing this repository, please use the following commands to verify you also included vcpkg to your local directory, since the dependencies will be downloaded and compiled locally inside vcpkg sub-folder.

```cmd
git clone --recursive git@github.com:lovoski/game_field.git
git submodule update --init --recursive
```

After ensuring you have vcpkg repository cloned to your local directory, please run the following command to bootstrap vcpkg manully, after this you should have an executable file named `vcpkg` under the vcpkg sub-folder:

```cmd
# linux users
./vcpkg/bootstrap-vcpkg.bat

# windows users
.\vcpkg\bootstrap-vcpkg.bat
```

Finally you can execute the following commands to have vcpkg download and compile the external dependencies, affter a while, you can use cmake extension in your IDE to configure the entire project, or configure the project with cmake manually. I would recommend vscode+cmake extesion.

```cmd
./vcpkg/vcpkg install assimp sdl2 glad zlib cgal bullet3
```
