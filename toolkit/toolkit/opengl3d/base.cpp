#include "toolkit/opengl3d/base.hpp"
#include <fstream>
#include <iostream>

namespace toolkit::opengl3d {

math::vector3 White = math::vector3(1.0, 1.0, 1.0);
math::vector3 Black = math::vector3(0.0, 0.0, 0.0);
math::vector3 Red = math::vector3(1.0, 0.0, 0.0);
math::vector3 Green = math::vector3(0.0, 1.0, 0.0);
math::vector3 Blue = math::vector3(0.0, 0.0, 1.0);
math::vector3 Yellow = math::vector3(1.0, 1.0, 0.0);
math::vector3 Purple = math::vector3(1.0, 0.0, 1.0);

void texture::create(GLenum target) {
  gl_handle = 0;
  gl_target = target;
  initialized = true;
  glGenTextures(1, &gl_handle);
  // sdl_context::get_instance().texture_handles.insert(gl_handle);
}
void texture::set_data_from_image(assets::image &img) {
  int nChannels = img.nchannels;
  if (nChannels == 3 || nChannels == 1)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  else
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
  GLint iformat = GL_R8;
  GLenum format = GL_RED;
  if (nChannels == 3) {
    iformat = GL_RGB8;
    format = GL_RGB;
  } else if (nChannels == 4) {
    iformat = GL_RGBA8;
    format = GL_RGBA;
  } else if (nChannels == 2) {
    iformat = GL_RG8;
    format = GL_RG;
  }
  set_data(img.width, img.height, iformat, format, GL_UNSIGNED_BYTE,
           img.data.data());
}

void buffer::create() {
  glGenBuffers(1, &gl_handle);
  // sdl_context::get_instance().buffer_handles.insert(gl_handle);
}

void vao::create() {
  glGenVertexArrays(1, &gl_handle);
  // sdl_context::get_instance().vertex_array_handles.insert(gl_handle);
}

void framebuffer::create() {
  glGenFramebuffers(1, &m_fbo);
  // sdl_context::get_instance().framebuffer_handles.insert(m_fbo);
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
  // sdl_context::get_instance().program_handles.insert(gl_handle);
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

  // sdl_context::get_instance().program_handles.insert(gl_handle);
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

bool create_image_from_texture(GLuint texture_handle, assets::image &img,
                               bool flip_vertical) {
  // Bind the texture
  glBindTexture(GL_TEXTURE_2D, texture_handle);

  // Get texture parameters: width, height, format
  int width, height, internalFormat;
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT,
                           &internalFormat);

  // Determine number of channels based on internalFormat
  int channels = 4;        // default to RGBA
  GLenum format = GL_RGBA; // default format
  switch (internalFormat) {
  case GL_RGB8:
  case GL_RGB:
    channels = 3;
    format = GL_RGB;
    break;
  case GL_RGBA8:
  case GL_RGBA:
    channels = 4;
    format = GL_RGBA;
    break;
  case GL_R8:
  case GL_RED:
    channels = 1;
    format = GL_RED;
    break;
  // Add more cases as needed
  default:
    // fallback
    channels = 4;
    format = GL_RGBA;
  }

  // Allocate buffer
  std::vector<unsigned char> pixels(width * height * channels);

  // Read the pixel data
  glGetTexImage(GL_TEXTURE_2D, 0, format, GL_UNSIGNED_BYTE, pixels.data());

  // Optionally flip vertically
  if (flip_vertical) {
    int rowSize = width * channels;
    std::vector<unsigned char> tempRow(rowSize);
    for (int y = 0; y < height / 2; ++y) {
      unsigned char *row1 = pixels.data() + y * rowSize;
      unsigned char *row2 = pixels.data() + (height - y - 1) * rowSize;
      std::copy(row1, row1 + rowSize, tempRow.begin());
      std::copy(row2, row2 + rowSize, row1);
      std::copy(tempRow.begin(), tempRow.end(), row2);
    }
  }

  // Fill the image struct
  img.resize(width, height, channels);
  img.data = std::move(pixels);
  return true;
}

}; // namespace toolkit::opengl