#pragma once

#include <glad/glad.h>

#include <iostream>
#include <fstream>
#include <array>

#include "toolkit/opengl3d/components/camera.hpp"
#include "toolkit/loaders/image.hpp"
#include "toolkit/system.hpp"

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include <SDL.h>
#include <SDL_opengl.h>

#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include <imgui.h>
#include <implot.h>

#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>

namespace toolkit::opengl3d {

extern math::vector3 White;
extern math::vector3 Black;
extern math::vector3 Red;
extern math::vector3 Green;
extern math::vector3 Blue;
extern math::vector3 Yellow;
extern math::vector3 Purple;

bool create_image_from_texture(GLuint texture_handle, assets::image &img,
                               bool flip_vertical = true);

class texture {
public:
  texture() {}
  ~texture() {}

  // Constructor: Initializes the texture object
  void create(GLenum target = GL_TEXTURE_2D);
  void set_data_from_image(assets::image &img);
  void create_from_image(assets::image &img) {
    create();
    set_data_from_image(img);
  }
  assets::image save_as_image() {
    assets::image img;
    create_image_from_texture(get_handle(), img);
    return img;
  }

  void del() {
    if (glIsTexture(gl_handle))
      glDeleteTextures(1, &gl_handle);
  }

  // Binds the texture to a specific texture unit (default: 0)
  void bind(GLuint unit = 0) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(gl_target, gl_handle);
  }

  // Unbinds the texture from the current target
  void unbind() const { glBindTexture(gl_target, 0); }

  // Sets the texture data and allocates storage
  void set_data(GLsizei width, GLsizei height, GLint internalFormat = GL_RGBA8,
                GLenum format = GL_RGBA, GLenum type = GL_UNSIGNED_BYTE,
                const void *data = nullptr) {
    m_width = width;
    m_height = height;
    m_internal_format = internalFormat;
    m_format = format;
    bind();
    glTexImage2D(gl_target, 0, internalFormat, width, height, 0, format, type,
                 data);
    unbind();
  }

  void clear_data() {
    static GLubyte whitePixel[4] = {255, 255, 255, 255};
    m_width = 1;
    m_height = 1;
    m_internal_format = GL_RGBA8;
    m_format = GL_RGBA;
    bind();
    glTexImage2D(gl_target, 0, m_internal_format, m_width, m_height, 0,
                 m_format, GL_UNSIGNED_BYTE, whitePixel);
    unbind();
  }

  // Configures texture parameters
  void set_parameters(const std::vector<std::pair<GLenum, GLint>> params) {
    bind();
    for (const auto &[pname, param] : params) {
      glTexParameteri(gl_target, pname, param);
    }
    unbind();
  }

  GLenum get_format() const { return m_format; }
  GLint get_internal_format() const { return m_internal_format; }

  // Returns the opengl texture ID
  GLuint get_handle() const { return gl_handle; }
  GLenum get_target() const { return gl_target; }

  // Returns the texture dimensions
  GLsizei get_width() const { return m_width; }
  GLsizei get_height() const { return m_height; }

  const bool inited() const { return initialized; }

private:
  bool initialized = false;

  GLuint gl_handle; // texture object ID
  GLenum gl_target; // texture target (e.g., GL_TEXTURE_2D)

  GLsizei m_width;         // texture width
  GLsizei m_height;        // texture height
  GLenum m_format;         // GL_RGBA
  GLint m_internal_format; // GL_RGBA8
};

// opengl buffer object
class buffer {
public:
  buffer() {}
  ~buffer() {}

  void create();
  void del() {
    if (glIsBuffer(gl_handle))
      glDeleteBuffers(1, &gl_handle);
  }

  // Bind the buffer to target, setup the filled data in it.
  template <typename T>
  void set_data_as(GLenum TARGET_BUFFER_NAME, const std::vector<T> &data,
                   GLenum usage = GL_STATIC_DRAW) {
    glBindBuffer(TARGET_BUFFER_NAME, gl_handle);
    // setting a buffer of size 0 is a invalid operation
    // use a buffer of size 1 byte with null datq instead
    if (data.size() == 0)
      glBufferData(TARGET_BUFFER_NAME, 1, nullptr, usage);
    else
      glBufferData(TARGET_BUFFER_NAME, data.size() * sizeof(T),
                   (void *)data.data(), usage);
  }
  template <typename T>
  void set_data_ssbo(const std::vector<T> &data,
                     GLenum usage = GL_DYNAMIC_DRAW) {
    set_data_as(GL_SHADER_STORAGE_BUFFER, data, usage);
  }
  void set_data_ssbo(unsigned int byteSize, GLenum usage = GL_DYNAMIC_DRAW) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_handle);
    if (byteSize == 0)
      glBufferData(GL_SHADER_STORAGE_BUFFER, 1, nullptr, usage);
    else
      glBufferData(GL_SHADER_STORAGE_BUFFER, byteSize, nullptr, usage);
  }
  // update the data in this buffer as described in offset. Make sure the buffer
  // has enough space for update.
  template <typename T>
  void update_data_as(GLenum TARGET_BUFFER_NAME, const std::vector<T> &data,
                      size_t offset) {
    glBindBuffer(TARGET_BUFFER_NAME, gl_handle);
    if (data.size() == 0)
      glBufferSubData(TARGET_BUFFER_NAME, offset, 1, nullptr);
    else
      glBufferSubData(TARGET_BUFFER_NAME, offset, data.size() * sizeof(T),
                      data.data());
  }
  template <typename T>
  void update_data_as(GLenum TARGET_BUFFER_NAME, const T data, size_t offset) {
    glBindBuffer(TARGET_BUFFER_NAME, gl_handle);
    glBufferSubData(TARGET_BUFFER_NAME, offset, sizeof(T), &data);
  }
  void copy_from(buffer &other, unsigned int byteSize) {
    other.bind_as(GL_COPY_READ_BUFFER);
    bind_as(GL_COPY_WRITE_BUFFER);
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
                        byteSize);
    glBindBuffer(GL_COPY_READ_BUFFER, 0);
    glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
  }
  void bind_as(GLenum TARGET_BUFFER_NAME) {
    glBindBuffer(TARGET_BUFFER_NAME, gl_handle);
  }
  // Bind SSBO, UBO to the bindingPoint of some shader, so that
  // these buffers can be accessed in this shader.
  void bind_to_point_as(GLenum TARGET_BUFFER_NAME, GLuint bindingPoint) {
    glBindBufferBase(TARGET_BUFFER_NAME, bindingPoint, gl_handle);
  }
  // unbind the buffer from target
  void unbind_as(GLenum TARGET_BUFFER_NAME) {
    glBindBuffer(TARGET_BUFFER_NAME, 0);
  }
  template <typename T> std::vector<T> map_data_as_vector(int elementSize) {
    T *ptr = (T *)map_data();
    std::vector<T> vec(elementSize);
    for (int i = 0; i < elementSize; i++)
      vec[i] = ptr[i];
    unmap_data();
    return vec;
  }
  template <typename T>
  std::vector<T> map_data_range_as_vector(unsigned int elementOffset,
                                          unsigned int elementSize) {
    T *ptr =
        (T *)map_data_range(elementOffset * sizeof(T), elementSize * sizeof(T));
    std::vector<T> vec(elementSize);
    for (int i = 0; i < elementSize; i++)
      vec[i] = ptr[i];
    unmap_data();
    return vec;
  }
  void *map_data(GLenum access = GL_READ_ONLY) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_handle);
    return glMapBuffer(GL_SHADER_STORAGE_BUFFER, access);
  }
  void *map_data_range(unsigned int offset, unsigned int byteSize,
                       GLbitfield access = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_handle);
    return glMapBufferRange(GL_SHADER_STORAGE_BUFFER, offset, byteSize, access);
  }
  void unmap_data() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_handle);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }
  GLuint get_handle() const { return gl_handle; }

private:
  GLuint gl_handle;
};

// opengl vertex array object
class vao {
public:
  vao() {}
  ~vao() {}

  void create();
  void del() {
    if (glIsVertexArray(gl_handle))
      glDeleteVertexArrays(1, &gl_handle);
  }

  void bind() const { glBindVertexArray(gl_handle); }
  void unbind() const { glBindVertexArray(0); }

  void link_attribute(buffer &vbo, GLuint layout, GLint size, GLenum type,
                      GLsizei stride, void *offset) {
    vbo.bind_as(GL_ARRAY_BUFFER);
    glVertexAttribPointer(layout, size, type, GL_FALSE, stride, offset);
    glEnableVertexAttribArray(layout);
    vbo.unbind_as(GL_ARRAY_BUFFER);
  }
  GLuint get_handle() const { return gl_handle; }

private:
  GLuint gl_handle;
};

class framebuffer {
public:
  framebuffer() {}
  ~framebuffer() {}

  void create();
  void del() {
    if (glIsFramebuffer(m_fbo))
      glDeleteFramebuffers(1, &m_fbo);
  }

  // record previous framebuffer, setup new framebuffer (you need to manually
  // setup viewport to render correctly)
  void bind(GLenum target = GL_FRAMEBUFFER) {
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &prev_fbo);
    glGetIntegerv(GL_VIEWPORT, prev_viewport);

    glBindFramebuffer(target, m_fbo);
    m_bound_target = target;
  }

  // Reset previous framebuffer and viewport
  void unbind(GLenum target = GL_FRAMEBUFFER) {
    glBindFramebuffer(GL_FRAMEBUFFER, prev_fbo);
    glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2],
               prev_viewport[3]);
  }

  void attach_color_buffer(const texture &texture,
                           GLenum attachment = GL_COLOR_ATTACHMENT0,
                           int mipLevel = 0) {
    glBindTexture(texture.get_target(), texture.get_handle());
    glFramebufferTexture2D(m_bound_target, attachment, texture.get_target(),
                           texture.get_handle(), mipLevel);
    attachments[attachment_counter++] = attachment;
  }

  void begin_draw_buffers() { attachment_counter = 0; }
  void end_draw_buffers() { glDrawBuffers(attachment_counter, attachments); }

  void attach_depth_buffer(const texture &texture) {
    glBindTexture(texture.get_target(), texture.get_handle());
    glFramebufferTexture2D(m_bound_target, GL_DEPTH_ATTACHMENT,
                           texture.get_target(), texture.get_handle(), 0);
  }

  bool check_status() {
    return glCheckFramebufferStatus(m_bound_target) == GL_FRAMEBUFFER_COMPLETE;
  }

  void set_viewport(int x, int y, int w, int h) const {
    glViewport(x, y, w, h);
  }

  unsigned int get_handle() { return m_fbo; }

private:
  GLuint m_fbo = 0;
  GLenum m_bound_target = GL_FRAMEBUFFER;
  unsigned int attachment_counter = 0;
  GLenum attachments[100];

  int prev_fbo, prev_viewport[4];
};

void check_compile_errors(GLuint shader, std::string type);

class shader {
public:
  std::string identifier;
  unsigned int gl_handle = 0;

  shader() {}
  ~shader() {}

  void del() {
    if (glIsProgram(gl_handle))
      glDeleteProgram(gl_handle);
  }

  // Load shader code directly, create and link program
  bool compile_shader_from_source(std::string vss, std::string fss,
                                  std::string gss = "none");

  // Load shader from path, compile and link them into a program
  bool compile_shader_from_path(std::string vsp, std::string fsp,
                                std::string gsp = "none");

  void use() { glUseProgram(gl_handle); }
  shader &set_bool(const std::string &name, bool value) {
    glUniform1i(glGetUniformLocation(gl_handle, name.c_str()), (int)value);
    return *this;
  }
  shader &set_int(const std::string &name, int value) {
    glUniform1i(glGetUniformLocation(gl_handle, name.c_str()), value);
    return *this;
  }
  shader &set_float(const std::string &name, float value) {
    glUniform1f(glGetUniformLocation(gl_handle, name.c_str()), value);
    return *this;
  }
  shader &set_vec2(const std::string &name, const math::vector2 &value) {
    glUniform2fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  shader &set_vec2(const std::string &name, float x, float y) {
    glUniform2f(glGetUniformLocation(gl_handle, name.c_str()), x, y);
    return *this;
  }
  shader &set_vec3(const std::string &name, const math::vector3 &value) {
    glUniform3fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  shader &set_vec3(const std::string &name, float x, float y, float z) {
    glUniform3f(glGetUniformLocation(gl_handle, name.c_str()), x, y, z);
    return *this;
  }
  shader &set_vec4(const std::string &name, const math::vector4 &value) {
    glUniform4fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  shader &set_vec4(const std::string &name, float x, float y, float z,
                   float w) {
    glUniform4f(glGetUniformLocation(gl_handle, name.c_str()), x, y, z, w);
    return *this;
  }
  shader &set_mat2(const std::string &name, const math::matrix2 &mat) {
    glUniformMatrix2fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }
  shader &set_mat3(const std::string &name, const math::matrix3 &mat) {
    glUniformMatrix3fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }
  shader &set_mat4(const std::string &name, const math::matrix4 &mat) {
    glUniformMatrix4fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }

  // Directly active a slot, set a texture by its integer handle
  bool set_texture2d(std::string name, unsigned int texture, int slot);

  shader &set_buffer_ssbo(buffer &buf, int binding) {
    buf.bind_to_point_as(GL_SHADER_STORAGE_BUFFER, binding);
    return *this;
  }

  bool set_cubemap(std::string name, unsigned int cubemapID, int slot);
};

class compute_shader {
public:
  std::string identifier;
  GLuint gl_handle;

  compute_shader() {}
  ~compute_shader() {}

  // Constructor reads and builds the compute shader
  void create(const std::string computeCode);
  void del() {
    if (glIsProgram(gl_handle))
      glDeleteProgram(gl_handle);
  }

  // Use the program
  void use() { glUseProgram(gl_handle); }
  compute_shader &set_bool(const std::string &name, bool value) {
    glUniform1i(glGetUniformLocation(gl_handle, name.c_str()), (int)value);
    return *this;
  }
  compute_shader &set_int(const std::string &name, int value) {
    glUniform1i(glGetUniformLocation(gl_handle, name.c_str()), value);
    return *this;
  }
  compute_shader &set_float(const std::string &name, float value) {
    glUniform1f(glGetUniformLocation(gl_handle, name.c_str()), value);
    return *this;
  }
  compute_shader &set_vec2(const std::string &name,
                           const math::vector2 &value) {
    glUniform2fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  compute_shader &set_vec2(const std::string &name, float x, float y) {
    glUniform2f(glGetUniformLocation(gl_handle, name.c_str()), x, y);
    return *this;
  }
  compute_shader &set_vec3(const std::string &name,
                           const math::vector3 &value) {
    glUniform3fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  compute_shader &set_vec3(const std::string &name, float x, float y, float z) {
    glUniform3f(glGetUniformLocation(gl_handle, name.c_str()), x, y, z);
    return *this;
  }
  compute_shader &set_vec4(const std::string &name,
                           const math::vector4 &value) {
    glUniform4fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                 value.data());
    return *this;
  }
  compute_shader &set_vec4(const std::string &name, float x, float y, float z,
                           float w) {
    glUniform4f(glGetUniformLocation(gl_handle, name.c_str()), x, y, z, w);
    return *this;
  }
  compute_shader &set_mat2(const std::string &name, const math::matrix2 &mat) {
    glUniformMatrix2fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }
  compute_shader &set_mat3(const std::string &name, const math::matrix3 &mat) {
    glUniformMatrix3fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }
  compute_shader &set_mat4(const std::string &name, const math::matrix4 &mat) {
    glUniformMatrix4fv(glGetUniformLocation(gl_handle, name.c_str()), 1,
                       GL_FALSE, mat.data());
    return *this;
  }

  compute_shader &bind_buffer(buffer &buffer, int binding) {
    buffer.bind_to_point_as(GL_SHADER_STORAGE_BUFFER, binding);
    return *this;
  }
  compute_shader &bind_buffer(unsigned int buffer, int binding) {
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, buffer);
    return *this;
  }

  // dispatch the compute shader
  void dispatch(GLuint numGroupsX, GLuint numGroupsY, GLuint numGroupsZ) {
    glUseProgram(gl_handle);
    glDispatchCompute(numGroupsX, numGroupsY, numGroupsZ);
  }

  void barrier(GLuint barrierBit = GL_SHADER_STORAGE_BARRIER_BIT) {
    glMemoryBarrier(barrierBit);
  }
};

}; // namespace toolkit::opengl