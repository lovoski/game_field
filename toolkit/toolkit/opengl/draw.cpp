#include "toolkit/opengl/draw.hpp"

using namespace toolkit::math;

namespace toolkit::opengl {

const std::string lineVS = R"(
  #version 330 core
  layout (location = 0) in vec3 pos;
  uniform mat4 vp;
  void main() {
    gl_Position = vp * vec4(pos, 1.0);
  }
  )";
const std::string lineFS = R"(
  #version 330 core
  uniform vec3 color;
  out vec4 FragColor;
  void main() {
    FragColor = vec4(color, 1.0);
  }
  )";

void draw_lines(std::vector<std::pair<vector3, vector3>> &lines, matrix4 vp,
                vector3 color) {
  static shader shader;
  static bool initialized = false;
  static vao vao;
  static buffer vbo;
  if (!initialized) {
    shader.compile_shader_from_source(lineVS, lineFS);
    initialized = true;
  }
  vao.bind();
  std::vector<vector3> points;
  for (auto &p : lines) {
    points.push_back(p.first);
    points.push_back(p.second);
  }
  vbo.set_data_as(GL_ARRAY_BUFFER, points);
  vao.link_attribute(vbo, 0, 3, GL_FLOAT, 3 * sizeof(float), (void *)0);
  shader.use();
  shader.set_mat4("vp", vp);
  shader.set_vec3("color", color);
  glDrawArrays(GL_LINES, 0, points.size());
  vbo.unbind_as(GL_ARRAY_BUFFER);
  vao.unbind();
}

void draw_linestrip(std::vector<vector3> &lineStrip, matrix4 vp,
                    vector3 color) {
  static shader shader;
  static bool initialized = false;
  static vao vao;
  static buffer vbo;
  if (!initialized) {
    shader.compile_shader_from_source(lineVS, lineFS);
    initialized = true;
  }
  vao.bind();
  vbo.set_data_as(GL_ARRAY_BUFFER, lineStrip);
  vao.link_attribute(vbo, 0, 3, GL_FLOAT, 3 * sizeof(float), (void *)0);
  shader.use();
  shader.set_mat4("vp", vp);
  shader.set_vec3("color", color);
  glDrawArrays(GL_LINE_STRIP, 0, lineStrip.size());
  vbo.unbind_as(GL_ARRAY_BUFFER);
  vao.unbind();
}
template <typename T> struct PlainVertex {
  T x, y, z;
  PlainVertex(T X, T Y, T Z) : x(X), y(Y), z(Z) {}
};
using PlainVertexi = PlainVertex<int>;
using PlainVertexf = PlainVertex<float>;
void draw_grid(unsigned int gridSize, unsigned int gridSpacing,
               math::matrix4 mvp, math::vector3 color) {
  static vao vao;
  static buffer vbo;
  static int savedGridSize = -1, savedGridSpacing = -1;
  static std::vector<PlainVertexf> points;
  static shader lineShader;
  static bool lineShaderLoaded = false;
  if (!lineShaderLoaded) {
    lineShader.compile_shader_from_source(lineVS, lineFS);
    lineShaderLoaded = true;
  }
  if (savedGridSize != gridSize || savedGridSpacing != gridSpacing) {
    // reallocate the data if grid size changes
    int current = gridSize;
    int spacing = gridSpacing;
    float sizef = static_cast<float>(gridSize);
    points.clear();
    while (current > 0) {
      float currentf = static_cast<float>(current);
      points.push_back({sizef, 0, currentf});
      points.push_back({-sizef, 0, currentf});
      points.push_back({sizef, 0, -currentf});
      points.push_back({-sizef, 0, -currentf});
      points.push_back({currentf, 0, sizef});
      points.push_back({currentf, 0, -sizef});
      points.push_back({-currentf, 0, sizef});
      points.push_back({-currentf, 0, -sizef});
      current -= spacing;
    }
    points.push_back({sizef, 0, 0});
    points.push_back({-sizef, 0, 0});
    points.push_back({0, 0, sizef});
    points.push_back({0, 0, -sizef});
    vao.bind();
    vbo.set_data_as(GL_ARRAY_BUFFER, points);
    vao.link_attribute(vbo, 0, 3, GL_FLOAT, 3 * sizeof(float), (void *)0);
    vao.unbind();
    vbo.unbind_as(GL_ARRAY_BUFFER);
    savedGridSize = gridSize;
    savedGridSpacing = gridSpacing;
  }
  lineShader.use();
  lineShader.set_vec3("color", 0.5 * color);
  lineShader.set_mat4("vp", mvp);
  vao.bind();
  // draw the normal grid lines
  glDrawArrays(GL_LINES, 0, points.size() - 4);
  // draw the axis lines
  lineShader.set_vec3("color", color);
  glDrawArrays(GL_LINES, points.size() - 4, 4);
  vao.unbind();
}

void draw_wire_sphere(math::vector3 position, math::matrix4 vp, float radius,
                      math::vector3 color, unsigned int segs) {
  math::vector3 walkDir1 = math::vector3(1.0f, 0.0f, 0.0f);
  math::vector3 walkDir2 = math::vector3(0.0f, 1.0f, 0.0f);
  math::vector3 walkDir3 = math::vector3(0.0f, 0.0f, 1.0f);
  std::vector<math::vector3> strip1{position + walkDir1 * radius};
  std::vector<math::vector3> strip2{position + walkDir2 * radius};
  std::vector<math::vector3> strip3{position + walkDir3 * radius};
  float rotDegree = math::deg_to_rad(360.0f / segs);
  for (int i = 0; i < segs; ++i) {
    walkDir1 =
        math::angle_axis(rotDegree * (i + 1), math::vector3(0.0f, 1.0f, 0.0f)) *
        math::vector3(1.0f, 0.0f, 0.0f);
    walkDir2 =
        math::angle_axis(rotDegree * (i + 1), math::vector3(0.0f, 0.0f, 1.0f)) *
        math::vector3(0.0f, 1.0f, 0.0f);
    walkDir3 =
        math::angle_axis(rotDegree * (i + 1), math::vector3(1.0f, 0.0f, 0.0f)) *
        math::vector3(0.0f, 0.0f, 1.0f);
    strip1.push_back(position + walkDir1 * radius);
    strip2.push_back(position + walkDir2 * radius);
    strip3.push_back(position + walkDir3 * radius);
  }
  draw_linestrip(strip1, vp, color);
  draw_linestrip(strip2, vp, color);
  draw_linestrip(strip3, vp, color);
}

void draw_arrow(math::vector3 start, math::vector3 end, math::matrix4 vp,
                math::vector3 color, float size) {
  static int segs = 12;
  math::vector3 dir = (end - start).normalized();
  math::vector3 normal =
      dir.cross(math::vector3(0.0f, 1.0f, 0.0f)).normalized();
  std::vector<math::vector3> strip1{start};
  std::vector<math::vector3> strip2{end + normal * size};
  float rotDegree = math::deg_to_rad(360.0f / segs);
  auto lastWalkDir = normal;
  for (int i = 0; i < segs; ++i) {
    auto walkDir = math::angle_axis(rotDegree * (i + 1), dir) * normal;
    strip1.push_back(end);
    strip1.push_back(end + lastWalkDir * size);
    strip1.push_back(end + dir * size * 1.8f);
    strip1.push_back(end + walkDir * size);
    strip2.push_back(end + walkDir * size);
    lastWalkDir = walkDir;
  }
  draw_linestrip(strip1, vp, color);
  draw_linestrip(strip2, vp, color);
}

void draw_cube(math::vector3 position, math::vector3 forward,
               math::vector3 left, math::vector3 up, math::matrix4 vp,
               math::vector3 size, math::vector3 color) {
  float fd = size.z(), ld = size.x(), ud = size.y();
  std::vector<math::vector3> strip1{position,
                                    position + left * ld,
                                    position + left * ld + forward * fd,
                                    position + forward * fd,
                                    position,
                                    position + up * ud,
                                    position + forward * fd + up * ud,
                                    position + forward * fd};
  std::vector<math::vector3> strip2{
      position + left * ld + up * ud,
      position + up * ud,
      position + forward * fd + up * ud,
      position + left * ld + forward * fd + up * ud,
      position + left * ld + up * ud,
      position + left * ld,
      position + left * ld + forward * fd,
      position + left * ld + forward * fd + up * ud};
  draw_linestrip(strip1, vp, color);
  draw_linestrip(strip2, vp, color);
}

std::string boneVS = R"(
  #version 330 core
  layout(location = 0) in vec3 aPos;
  
  void main() {
      gl_Position = vec4(aPos, 1.0);
  }
  )";
std::string boneGS = R"(
  #version 330 core
  layout(lines) in;
  layout(line_strip, max_vertices = 24) out;
  
  uniform mat4 mvp;
  uniform float radius;
  
  void main() {
      vec3 start = gl_in[0].gl_Position.xyz;
      vec3 end = gl_in[1].gl_Position.xyz;
  
      // Compute direction and orthogonal vectors
      vec3 dir = end - start;
      float boneLength = length(dir);
      float normalOffset = radius * boneLength;
      vec3 direction = normalize(dir);
      vec3 up = normalize(vec3(0.0, 1.0, 1e-3));
      vec3 right = normalize(cross(direction, up));
      float dirOffset = 0.17;
      up = cross(right, direction);
  
      // Define the four offset vectors for the octahedral shape
      vec3 offsets[4];
      offsets[0] = right * normalOffset + dirOffset * dir;
      offsets[1] = up * normalOffset + dirOffset * dir;
      offsets[2] = -right * normalOffset + dirOffset * dir;
      offsets[3] = -up * normalOffset + dirOffset * dir;
  
      // Generate the edges of the octahedron
      for (int i = 0; i < 4; ++i) {
          // Lines connecting start point to the offset points
          gl_Position = mvp * vec4(start, 1.0);
          EmitVertex();
          gl_Position = mvp * vec4(start + offsets[i], 1.0);
          EmitVertex();
          EndPrimitive();
  
          gl_Position = mvp * vec4(start + offsets[(i+1)%4], 1.0);
          EmitVertex();
          gl_Position = mvp * vec4(start + offsets[i], 1.0);
          EmitVertex();
          EndPrimitive();
  
          // Lines connecting the start and end offset points
          gl_Position = mvp * vec4(start + offsets[i], 1.0);
          EmitVertex();
          gl_Position = mvp * vec4(end, 1.0);
          EmitVertex();
          EndPrimitive();
      }
  }
  )";
std::string boneFS = R"(
  #version 330 core
  out vec4 FragColor;
  
  uniform vec3 color; // Color of the wireframe
  
  void main() {
      FragColor = vec4(color, 1.0);
  }
  )";
void draw_bones(std::vector<std::pair<math::vector3, math::vector3>> &bones,
                math::vector2 viewport, math::matrix4 vp, math::vector3 color) {
  static vao vao;
  static buffer vbo;
  static shader boneShader;
  static bool shaderInitialized = false;
  if (!shaderInitialized) {
    boneShader.compile_shader_from_source(boneVS, boneFS, boneGS);
    shaderInitialized = true;
  }
  // initialize vbo with `bones`
  vao.bind();
  std::vector<math::vector3> buffer;
  for (auto &pair : bones) {
    buffer.push_back(pair.first);  // bone start
    buffer.push_back(pair.second); // bond end
  }
  vbo.set_data_as(GL_ARRAY_BUFFER, buffer);
  vao.link_attribute(vbo, 0, 3, GL_FLOAT, 3 * sizeof(float), (void *)0);
  boneShader.use();

  boneShader.set_float("radius", 0.1f);
  boneShader.set_vec3("color", color);
  boneShader.set_vec2("viewportSize", viewport);
  boneShader.set_mat4("mvp", vp);

  glDrawArrays(GL_LINES, 0, buffer.size());

  vao.unbind();
  vbo.unbind_as(GL_ARRAY_BUFFER);
}

struct _planevertex {
  math::vector4 position;
  math::vector4 normal;
  math::vector4 texcoords;
};

void quad_draw_call() {
  static vao quad;
  static buffer quadVBO;
  static bool initialized = false;
  if (!initialized) {
    quad.create();
    quadVBO.create();
    float quadVertices[] = {
        // positions        // texture coordinates
        1.0f,  -1.0f, 0.0f, 1.0f, 0.0f, // Bottom-right
        1.0f,  1.0f,  0.0f, 1.0f, 1.0f, // Top-right
        -1.0f, 1.0f,  0.0f, 0.0f, 1.0f, // Top-left

        -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, // Bottom-left
        1.0f,  -1.0f, 0.0f, 1.0f, 0.0f, // Bottom-right
        -1.0f, 1.0f,  0.0f, 0.0f, 1.0f, // Top-left
    };
    quad.bind();
    quadVBO.bind_as(GL_ARRAY_BUFFER);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices,
                 GL_STATIC_DRAW);

    quad.link_attribute(quadVBO, 0, 3, GL_FLOAT, 5 * sizeof(float), (void *)0);
    quad.link_attribute(quadVBO, 1, 2, GL_FLOAT, 5 * sizeof(float),
                        (void *)(3 * sizeof(float)));
    quad.unbind();
    quadVBO.unbind_as(GL_ARRAY_BUFFER);
    initialized = true;
  }
  quad.bind();
  glDrawArrays(GL_TRIANGLES, 0, 6);
  quad.unbind();
}

}; // namespace toolkit::opengl