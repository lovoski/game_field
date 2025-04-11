#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#include <imgui.h>
#include <implot.h>
#include <ImGuizmo.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <toolkit/math.hpp>
#include <toolkit/utils.hpp>
#include <toolkit/loaders/image.hpp>

#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>