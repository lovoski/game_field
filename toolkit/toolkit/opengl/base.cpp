#include "toolkit/opengl/base.hpp"
#include "toolkit/assets/fonts.hpp"

namespace toolkit::opengl {

math::vector3 White = math::vector3(1.0, 1.0, 1.0);
math::vector3 Black = math::vector3(0.0, 0.0, 0.0);
math::vector3 Red = math::vector3(1.0, 0.0, 0.0);
math::vector3 Green = math::vector3(0.0, 1.0, 0.0);
math::vector3 Blue = math::vector3(0.0, 0.0, 1.0);
math::vector3 Yellow = math::vector3(1.0, 1.0, 0.0);
math::vector3 Purple = math::vector3(1.0, 0.0, 1.0);

void context::init(unsigned int width, unsigned int height, const char *title,
                   int majorVersion, int minorVersion) {
  wnd_width = width;
  wnd_height = height;
  scene_width = width;
  scene_height = height;

  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, majorVersion);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, minorVersion);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  window = glfwCreateWindow(width, height, title, NULL, NULL);
  if (!window) {
    printf("Failed to create opengl window version %d.%d\n", majorVersion,
           minorVersion);
    return;
  }

  glfwMakeContextCurrent(window);

  glfwSetCursorPosCallback(window, context::cursor_pos_callback);
  glfwSetScrollCallback(window, context::scroll_callback);
  glfwSetMouseButtonCallback(window, context::mouse_button_callback);
  glfwSetKeyCallback(window, context::key_callback);

  glfwSetFramebufferSizeCallback(window, context::framebuffer_resize_callback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    printf("Failed to load glad\n");
    return;
  }

  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init();
  ImPlot::CreateContext();

  ImGui::GetIO().Fonts->AddFontFromMemoryCompressedTTF(
      cascadia_code_yahei_data, cascadia_code_yahei_size, 16.0f, nullptr,
      ImGui::GetIO().Fonts->GetGlyphRangesChineseFull());
}

void context::shutdown() {
  // free opengl handles
  for (auto handle : buffer_handles)
    glDeleteBuffers(1, &handle);
  for (auto handle : vertex_array_handles)
    glDeleteVertexArrays(1, &handle);
  for (auto handle : texture_handles)
    glDeleteTextures(1, &handle);
  for (auto handle : program_handles)
    glDeleteProgram(handle);
  for (auto handle : framebuffer_handles)
    glDeleteFramebuffers(1, &handle);

  window = nullptr;
  ImPlot::DestroyContext();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  glfwDestroyWindow(window);
  glfwTerminate();
}

void context::run(std::function<void(void)> mainLoop) {
  while (!glfwWindowShouldClose(window)) {
    poll();
    mainLoop();
  }
}

void context::poll() {
  scroll_offset << 0.0, 0.0;
  triggered_keys.clear();
  untriggered_keys.clear();
  triggered_mouse_keys.clear();
  untriggered_mouse_keys.clear();
  glfwPollEvents();
}

void context::begin_imgui() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void context::end_imgui() {
  ImGui::EndFrame();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool context::is_key_pressed(int key) const {
  auto it = key_states.find(key);
  return it != key_states.end() && it->second;
}
bool context::is_key_triggered(int key) const {
  return triggered_keys.count(key) != 0;
}
bool context::is_key_untriggered(int key) const {
  return untriggered_keys.count(key) != 0;
}
bool context::is_mouse_button_triggered(int key) const {
  return triggered_mouse_keys.count(key) != 0;
}
bool context::is_mouse_button_untriggered(int key) const {
  return untriggered_mouse_keys.count(key) != 0;
}
bool context::is_mouse_button_pressed(int button) const {
  auto it = mouse_button_states.find(button);
  return it != mouse_button_states.end() && it->second;
}

bool context::cursor_in_scene_window() {
  return mouse_x >= scene_pos_x && mouse_x <= scene_pos_x + scene_width &&
         mouse_y >= scene_pos_y && mouse_y <= scene_pos_y + scene_height;
}

bool context::loop_cursor_in_window() {
  bool loop = false;
  while (mouse_x < 0) {
    mouse_x += wnd_width;
    loop = true;
  }
  while (mouse_x > wnd_width) {
    mouse_x -= wnd_width;
    loop = true;
  }
  while (mouse_y < 0) {
    mouse_y += wnd_height;
    loop = true;
  }
  while (mouse_y > wnd_height) {
    mouse_y -= wnd_height;
    loop = true;
  }
  glfwSetCursorPos(window, mouse_x, mouse_y);
  return loop;
}

// Load shader from path, compile and link them into a program
bool shader::compile_shader_from_path(std::string vsp, std::string fsp,
                                      std::string gsp) {
  // delete previous program if exists
  if (gl_handle != 0) {
    // free old shader if there's any
    glDeleteProgram(gl_handle);
  }
  std::string vertexCode;
  std::string fragmentCode;
  std::string geometryCode = "none";
  try {
    std::ifstream vin(vsp), fin(fsp);
    if (!vin.is_open() || !fin.is_open()) {
      printf("failed to open shader file at %s and %s\n", vsp.c_str(),
             fsp.c_str());
      return false;
    }
    std::stringstream vinss, finss;
    vinss << vin.rdbuf();
    finss << fin.rdbuf();
    vertexCode = vinss.str();
    fragmentCode = finss.str();
    // if geometry shader path is present, also load a geometry shader
    if (gsp != "none") {
      std::ifstream gin(gsp);
      if (!gin.is_open()) {
        printf("failed to open shader file at %s\n", gsp.c_str());
        return false;
      }
      std::stringstream ginss;
      ginss << gin.rdbuf();
      geometryCode = ginss.str();
    }
  } catch (std::ifstream::failure &e) {
    printf("failed to load shader from path, %s\n", e.what());
    return false;
  }
  return compile_shader_from_source(vertexCode, fragmentCode, geometryCode);
}

// Load shader code directly, create and link program
bool shader::compile_shader_from_source(std::string vss, std::string fss,
                                        std::string gss) {
  unsigned int vertex, fragment;
  // vertex shader
  const char *vShaderCode = vss.c_str();
  const char *fShaderCode = fss.c_str();
  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &vShaderCode, NULL);
  glCompileShader(vertex);
  check_compile_errors(vertex, "VERTEX");
  // fragment Shader
  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &fShaderCode, NULL);
  glCompileShader(fragment);
  check_compile_errors(fragment, "FRAGMENT");
  // if geometry shader is given, compile geometry shader
  unsigned int geometry;
  if (gss != "none") {
    const char *gShaderCode = gss.c_str();
    geometry = glCreateShader(GL_GEOMETRY_SHADER);
    glShaderSource(geometry, 1, &gShaderCode, NULL);
    glCompileShader(geometry);
    check_compile_errors(geometry, "GEOMETRY");
  }
  // shader Program
  gl_handle = glCreateProgram();
  glAttachShader(gl_handle, vertex);
  glAttachShader(gl_handle, fragment);
  if (gss != "none")
    glAttachShader(gl_handle, geometry);
  // linking the shader is a time consuming process
  // this should be avoglHandle between frames in all cost
  glLinkProgram(gl_handle);
  check_compile_errors(gl_handle, "PROGRAM");
  // delete the shaders as they're linked into our program now and no longer
  // necessary
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  if (gss != "none")
    glDeleteShader(geometry);
  context::program_handles.insert(gl_handle);
  return true;
}

bool shader::set_texture2d(std::string name, unsigned int texture, int slot) {
  use();
  glActiveTexture(GL_TEXTURE0 + slot);
  int location = glGetUniformLocation(gl_handle, name.c_str());
  if (location == -1) {
    // LOG_F(WARNING, "location for %s not valglHandle", name.c_str());
    return false;
  }
  glUniform1i(location, slot);
  glBindTexture(GL_TEXTURE_2D, texture);
  return true;
}

bool shader::set_cubemap(std::string name, unsigned int cubemapID, int slot) {
  use();
  glActiveTexture(GL_TEXTURE0 + slot);
  int location = glGetUniformLocation(gl_handle, name.c_str());
  if (location == -1)
    return false;
  glUniform1i(location, slot);
  glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapID);
  return true;
}

void compute_shader::create(const std::string computeCode) {
  const char *cShaderCode = computeCode.c_str();
  // Compile shaders
  GLuint compute;
  GLint success;
  GLchar infoLog[1024];

  // Compute Shader
  compute = glCreateShader(GL_COMPUTE_SHADER);
  glShaderSource(compute, 1, &cShaderCode, nullptr);
  glCompileShader(compute);
  check_compile_errors(compute, "COMPUTE");

  // Shader Program
  gl_handle = glCreateProgram();
  glAttachShader(gl_handle, compute);
  glLinkProgram(gl_handle);
  check_compile_errors(gl_handle, "PROGRAM");

  // del the shader as it's linked into our program now and no longer
  // necessary
  glDeleteShader(compute);

  context::program_handles.insert(gl_handle);
}

void check_compile_errors(GLuint shader, std::string type) {
  GLint success;
  GLchar infoLog[1024];
  if (type != "PROGRAM") {
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(shader, 1024, NULL, infoLog);
      printf("SHADER_COMPILATION_ERROR of type: %s\n %s", type.c_str(),
             infoLog);
    }
  } else {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(shader, 1024, NULL, infoLog);
      printf("PROGRAM_LINKING_ERROR of type: %s\n %s", type.c_str(), infoLog);
    }
  }
}

}; // namespace toolkit::opengl