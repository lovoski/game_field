#include "app.hpp"
#include <codecvt>
#include <filesystem>
#include <locale>

using namespace toolkit;

/*
The page has the same ratio as the loaded page, width=w/h, height = 1.0.
Centered at origin.

The heigh for this generated page is fixed to 1.0, but the width may vary.
 */
std::string createPageGeometry = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE, local_size_y = WORK_GROUP_SIZE, local_size_z = 1) in;

struct vert_data {
  vec4 pos;
  vec4 texCoord;
};
layout(std430, binding = 0) buffer VertexBuffer {
  vert_data gVertex[];
};

uniform float gGridCellSizeX;
uniform float gGridCellSizeY;

uniform int gGridPointHorizontal;
uniform int gGridPointVertical;

void main() {
  ivec2 vertIdx = ivec2(gl_GlobalInvocationID.xy);
  if (vertIdx.x >= gGridPointHorizontal || vertIdx.y >= gGridPointVertical) return;
  vec3 basicPos = vec3(gGridCellSizeX * vertIdx.x, gGridCellSizeY * vertIdx.y, 0.0);

  vert_data vd;
  vd.pos = vec4(basicPos, 1.0);
  vd.texCoord.xy = vec2(vertIdx.x/float(gGridPointHorizontal-1), 1.0-vertIdx.y/float(gGridPointVertical-1));
  gVertex[gGridPointHorizontal * vertIdx.y + vertIdx.x] = vd;
}
)";

/*
Assume the input page always have height=1.0 and varied width.
*/
std::string curlPageGeometry = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE, local_size_y = WORK_GROUP_SIZE, local_size_z = 1) in;

struct vert_data {
  vec4 pos;
  vec4 texCoord;
};
layout(std430, binding = 0) buffer VertexBuffer {
  vert_data gVertex[];
};
layout(std430, binding = 1) buffer DeformedVertexBuffer {
  vert_data gDeformedVertex[];
};

uniform float gGridCellSizeX;
uniform float gGridCellSizeY;

uniform int gGridPointHorizontal;
uniform int gGridPointVertical;

uniform bool gPageFromRightToLeft;

uniform float gFoldK;
uniform float gFoldB;
uniform float gFoldKInterpStart;
uniform float gFoldKInterpEnd;

const float bMargin = 0;
const float pi = 3.1415926535;
const float R = 0.05;

void main() {
  ivec2 vertIdx = ivec2(gl_GlobalInvocationID.xy);
  if (vertIdx.x >= gGridPointHorizontal || vertIdx.y >= gGridPointVertical) return;
  int index = gGridPointHorizontal * vertIdx.y + vertIdx.x;
  float width = gGridCellSizeX * (gGridPointHorizontal-1);
  float height = gGridCellSizeY * (gGridPointVertical-1);
  vert_data vd = gVertex[index];
  vec3 p = vd.pos.xyz;
  int pageTurnBit;
  if (!gPageFromRightToLeft) {
    p.x -= width;
    pageTurnBit = -1;
  } else {
    pageTurnBit = 1;
  }
  float k = gFoldK;
  float b = gFoldB;
  // map `k` value
  k = sign(k)*min(abs(k), gFoldKInterpEnd);
  float theta = 0;
  if (abs(k) > gFoldKInterpStart) {
    theta = pi*(abs(k)-gFoldKInterpStart)/(gFoldKInterpEnd-gFoldKInterpStart);
  }
  // clamp `b` value
  if (k*pageTurnBit>0) {
    if (b > -height*bMargin) {
      b = -height*bMargin;
    }
  } else {
    if (b < height*(1+bMargin)) {
      b = height*(1+bMargin);
    }
  }

  float tx, ty, tz, tw = 1.0;
  float r = R;
  float cx = (p.x+(p.y-b)*k)/(1+k*k);
  float cy = k*cx+b;
  float dist = length(vec2(cx, cy)-p.xy);
  bool noFold = pageTurnBit*k*(k*p.x+b-p.y) <= 0;
  if (noFold) {
    tx = p.x;
    ty = p.y;
    tz = p.z;
  } else {
    if (dist < (pi-theta)*r) {
      float alpha = dist/r;
      float relativeDist = r*(sin(alpha+theta)-sin(theta));
      float m = sign(relativeDist)*sqrt(pow(relativeDist,2)*k*k/(k*k+1))*pageTurnBit;
      tx = cx+m;
      ty = cy-m/k;
      tz = r*(cos(theta)-cos(alpha+theta));
      tw = abs(cos(alpha+theta));
    } else {
      float relativeDist = -(dist-(pi-theta-sin(theta))*r);
      float m = sign(relativeDist)*sqrt(pow(relativeDist,2)*k*k/(1+k*k))*pageTurnBit;
      tx = cx+m;
      ty = cy-m/k;
      tz = r*(1+cos(theta));
    }
  }

  // interpolate between the final position and current position
  // to ensure the page folds to target in the end
  float cxr0 = abs(b/(abs(k)+1e-5))/width;
  float cxr1 = abs((height-b)/(abs(k)+1e-5))/width;
  if (min(cxr0, cxr1) <= 0.15) {
    vec3 tp = vec3(-p.x, p.y, p.z);
    float alpha = clamp(pow(abs(k)/gFoldKInterpEnd, 2), 0, 1);
    tx = alpha*tp.x+(1-alpha)*tx;
    ty = alpha*tp.y+(1-alpha)*ty;
    tz = alpha*tp.z+(1-alpha)*tz;
  }

  vd.pos = vec4(tx, ty, tz, tw);
  gDeformedVertex[index] = vd;
  vd.texCoord.x = 1.0-vd.texCoord.x;
  vd.texCoord += vec4(2.0, 2.0, 0.0, 0.0);
  gDeformedVertex[index + gGridPointHorizontal * gGridPointVertical] = vd;
}
)";

std::string pageVS = R"(
#version 430
layout(location = 0) in vec4 aPos;
layout(location = 1) in vec4 aTexCoord;

out vec2 texCoord;
out float curlFactor;

uniform float gOffsetX;
uniform mat4 gVP;

void main() {
  texCoord = aTexCoord.xy;
  curlFactor = aPos.w;
  vec3 pos = aPos.xyz;
  pos.x += gOffsetX;
  gl_Position = gVP * vec4(pos, 1.0);
}
)";

std::string pageFS = R"(
#version 430

in vec2 texCoord;
in float curlFactor;

uniform sampler2D gPageTex;
uniform sampler2D gBackPageTex;
uniform bool gColored;
uniform bool gBackColored;

uniform vec3 gEyeProtectionColor;

out vec4 FragColor;

void main() {
  vec4 pageColor;
  vec2 uv;
  if (texCoord.x > 1.5) {
    uv = texCoord-vec2(2.0);
    pageColor = texture(gBackPageTex, uv);
    if (!gBackColored)
      pageColor = vec4(vec3(pageColor.r), 1.0);
  } else {
    uv = texCoord;
    pageColor = texture(gPageTex, uv);
    if (!gColored)
      pageColor = vec4(vec3(pageColor.r), 1.0);
  }
  float mx = 2*uv.x-1, my = 2*uv.y-1;
  float mx4 = mx*mx*mx*mx, my4 = my*my*my*my;
  float mx16 = mx4*mx4*mx4*mx4, my16 = my4*my4*my4*my4;
  float mx64 = mx16*mx16*mx16*mx16, my64 = my16*my16*my16*my16;
  float edgeFactor = (mx64-1)*(my64-1);
  float mixFactor = mix(0.95, curlFactor*0.5+0.5, edgeFactor);
  vec3 color = mixFactor * pageColor.xyz * gEyeProtectionColor;
  FragColor = vec4(color, 1.0);
}
)";

struct vert_data {
  math::vector4 pos;
  math::vector4 texCoord;
};

/**
 * Get the index buffer for triangle grid with `x` horizontal grid points and
 * `y` vertical grid points.
 */
std::vector<unsigned int> getGridIndexBuffer(unsigned int x, unsigned y) {
  std::vector<unsigned int> result((x - 1) * (y - 1) * 6, 0);
  for (int i = 0; i < x - 1; i++) {
    for (int j = 0; j < y - 1; j++) {
      int k = i + j * x;
      int offset = (k - j) * 6;
      result[offset + 0] = k;
      result[offset + 1] = k + 1;
      result[offset + 2] = k + x;
      result[offset + 3] = k + x;
      result[offset + 4] = k + 1;
      result[offset + 5] = k + 1 + x;
    }
  }
  return result;
}

void manga_viewer::resizePageGrid() {
  createPageGeometryProgram.create(
      toolkit::str_format(createPageGeometry.c_str(), 8));
  deformPageGeometryProgram.create(
      toolkit::str_format(curlPageGeometry.c_str(), 8));
  pageIndexBuffer.create();
  pageVertexBuffer.create();
  deformedPageVertexBuffer.create();
  whiteTexture.create();
  whiteTexture.clear_data();

  auto tmpIndices = getGridIndexBuffer(gridPointHorizontal, gridPointVertical);
  unsigned int numIndices = tmpIndices.size();
  std::vector<unsigned int> indices(numIndices * 2);
  for (int i = 0; i < numIndices * 2; i++) {
    if (i < numIndices) {
      indices[i] = tmpIndices[i];
    } else {
      int mod = (i - numIndices) % 3, index;
      if (mod == 0)
        index = i - numIndices;
      else if (mod == 1)
        index = i - numIndices + 1;
      else
        index = i - numIndices - 1;
      indices[i] = tmpIndices[index] + gridPointHorizontal * gridPointVertical;
    }
  }
  pageIndexBuffer.set_data_ssbo(indices);

  pageVertexBuffer.set_data_ssbo(
      std::vector<vert_data>(gridPointHorizontal * gridPointVertical));
  deformedPageVertexBuffer.set_data_ssbo(
      std::vector<vert_data>(gridPointHorizontal * gridPointVertical * 2));
}

void manga_viewer::draw_book() {
  applyHighResQueue();

  static toolkit::opengl::vao vao;
  static toolkit::opengl::shader shader;
  static bool initialized = false;
  if (!initialized) {
    vao.create();
    shader.compile_shader_from_source(pageVS, pageFS);
    initialized = true;
  }

  // limit the range of leadingPageIdx
  leadingPageIdx =
      std::clamp(leadingPageIdx, -1,
                 pageCount.load() + (pageCount.load() % 2 == 0 ? -1 : 0));

  // create page geometry
  float pageHeight = 1.0f;
  float pageWidth = first_page_width_div_height;
  gridCellSizeY = 1.0f / (gridPointVertical - 1);
  gridCellSizeX = gridCellSizeY * first_page_width_div_height;
  createPageGeometryProgram.use();
  createPageGeometryProgram.set_float("gGridCellSizeX", gridCellSizeX);
  createPageGeometryProgram.set_float("gGridCellSizeY", gridCellSizeY);
  createPageGeometryProgram.set_int("gGridPointHorizontal",
                                    gridPointHorizontal);
  createPageGeometryProgram.set_int("gGridPointVertical", gridPointVertical);
  createPageGeometryProgram.bind_buffer(pageVertexBuffer.get_handle(), 0);
  createPageGeometryProgram.dispatch((gridPointHorizontal + 7) / 8,
                                     (gridPointVertical + 7) / 8, 1);
  createPageGeometryProgram.barrier(GL_SHADER_STORAGE_BARRIER_BIT |
                                    GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

  bool turnToNextPage = (pageFromRightToLeft && pageFlowRTL) ||
                        (!pageFromRightToLeft && !pageFlowRTL);
  bool turnToPreviousPage = (!pageFromRightToLeft && pageFlowRTL) ||
                            (pageFromRightToLeft && !pageFlowRTL);
  if (autoTurnPage) {
    if (std::abs(foldK) >= foldKInterpEnd) {
      // stop auto turn page
      autoTurnPage = false;
      // reset loadingPageIdx
      if (turnToNextPage)
        leadingPageIdx += 2;
      else
        leadingPageIdx -= 2;
    } else {
      // tune parameters automatically
      float alpha = std::pow(
          std::clamp(std::abs(foldK) / foldKInterpEnd, 0.0f, 1 - 1e-3f), 1);
      float delta = (alpha * 0.1 * autoTurnPageSpeed +
                     (1 - alpha) * 0.7 * autoTurnPageSpeed) *
                    deltaTime * (pageFromRightToLeft ? 1 : -1);
      float newFoldK =
          (foldK + std::tan(delta)) / (1 - foldK * std::tan(delta));
      foldK = newFoldK * foldK > 0
                  ? newFoldK
                  : (foldKInterpEnd + 1e-3) * (pageFromRightToLeft ? 1 : -1);
    }
  }

  // the deformed page won't get rendered when:
  // 1. not at autoTurnPage state
  // 2. trying to turn previous page at the front page
  // 3. trying to turn next page at the ending page
  bool renderDeformedPage =
      autoTurnPage &&
      !(leadingPageIdx >= pageCount.load() - 1 && turnToNextPage) &&
      !(leadingPageIdx <= -1 && turnToPreviousPage);

  // render the leading background page
  int leadingBackgroundPageIdx;
  if (renderDeformedPage) {
    if (turnToNextPage)
      leadingBackgroundPageIdx = leadingPageIdx;
    else
      leadingBackgroundPageIdx = leadingPageIdx - 2;
  } else {
    leadingBackgroundPageIdx = leadingPageIdx;
  }
  if (leadingPageIdx != -1 &&
      !(renderDeformedPage && leadingPageIdx == 1 && turnToPreviousPage)) {
    vao.bind();
    pageVertexBuffer.bind_as(GL_ARRAY_BUFFER);
    pageIndexBuffer.bind_as(GL_ELEMENT_ARRAY_BUFFER);
    vao.link_attribute(pageVertexBuffer, 0, 4, GL_FLOAT, sizeof(vert_data), 0);
    vao.link_attribute(pageVertexBuffer, 1, 4, GL_FLOAT, sizeof(vert_data),
                       (void *)(offsetof(vert_data, texCoord)));
    shader.use();
    shader.set_float("gOffsetX", pageFlowRTL ? -pageWidth : 0);
    shader.set_mat4("gVP", vp);
    shader.set_vec3("gEyeProtectionColor",
                    eyeProtection ? eyeProtectionColor : math::vector3::Ones());
    shader.set_bool(
        "gColored",
        leadingBackgroundPageIdx >= pageCount.load()
            ? whiteTexture.get_format() != GL_RED
            : getTextureFromPool(leadingBackgroundPageIdx).get_format() !=
                  GL_RED);
    shader.set_texture2d(
        "gPageTex",
        (leadingBackgroundPageIdx >=
         pageCount.load() + (padAfterFirstPage ? 1 : 0))
            ? whiteTexture.get_handle()
            : getTextureFromPool(leadingBackgroundPageIdx).get_handle(),
        0);
    glDrawElements(GL_TRIANGLES,
                   (gridPointVertical - 1) * (gridPointHorizontal - 1) * 6,
                   GL_UNSIGNED_INT, 0);
    vao.unbind();
  }

  // render the following background page if valid
  int followingBackgroundPageIdx;
  if (renderDeformedPage) {
    if (turnToNextPage)
      followingBackgroundPageIdx = leadingPageIdx + 3;
    else
      followingBackgroundPageIdx = leadingPageIdx + 1;
  } else {
    followingBackgroundPageIdx = leadingPageIdx + 1;
  }
  if (!(followingBackgroundPageIdx >=
        (pageCount.load() + (padAfterFirstPage ? 1 : 0)))) {
    vao.bind();
    pageVertexBuffer.bind_as(GL_ARRAY_BUFFER);
    pageIndexBuffer.bind_as(GL_ELEMENT_ARRAY_BUFFER);
    vao.link_attribute(pageVertexBuffer, 0, 4, GL_FLOAT, sizeof(vert_data), 0);
    vao.link_attribute(pageVertexBuffer, 1, 4, GL_FLOAT, sizeof(vert_data),
                       (void *)(offsetof(vert_data, texCoord)));
    shader.use();
    shader.set_float("gOffsetX", pageFlowRTL ? 0 : -pageWidth);
    shader.set_mat4("gVP", vp);
    shader.set_vec3("gEyeProtectionColor",
                    eyeProtection ? eyeProtectionColor : math::vector3::Ones());
    shader.set_bool(
        "gColored",
        (followingBackgroundPageIdx >=
         (pageCount + (padAfterFirstPage ? 1 : 0)))
            ? whiteTexture.get_format() != GL_RED
            : getTextureFromPool(followingBackgroundPageIdx).get_format() !=
                  GL_RED);
    shader.set_texture2d(
        "gPageTex",
        (followingBackgroundPageIdx >=
         (pageCount + (padAfterFirstPage ? 1 : 0)))
            ? whiteTexture.get_handle()
            : getTextureFromPool(followingBackgroundPageIdx).get_handle(),
        0);
    glDrawElements(GL_TRIANGLES,
                   (gridPointVertical - 1) * (gridPointHorizontal - 1) * 6,
                   GL_UNSIGNED_INT, 0);
    vao.unbind();
  }

  if (renderDeformedPage) {
    // deform page geometry
    deformPageGeometryProgram.use();
    deformPageGeometryProgram.set_bool("gPageFromRightToLeft",
                                       pageFromRightToLeft);
    deformPageGeometryProgram.set_float("gFoldK", foldK);
    deformPageGeometryProgram.set_float("gFoldB", foldB);
    deformPageGeometryProgram.set_float("gFoldKInterpStart", foldKInterpStart);
    deformPageGeometryProgram.set_float("gFoldKInterpEnd", foldKInterpEnd);
    deformPageGeometryProgram.set_float("gGridCellSizeX", gridCellSizeX);
    deformPageGeometryProgram.set_float("gGridCellSizeY", gridCellSizeY);
    deformPageGeometryProgram.set_int("gGridPointHorizontal",
                                      gridPointHorizontal);
    deformPageGeometryProgram.set_int("gGridPointVertical", gridPointVertical);
    deformPageGeometryProgram.bind_buffer(pageVertexBuffer.get_handle(), 0)
        .bind_buffer(deformedPageVertexBuffer.get_handle(), 1);
    deformPageGeometryProgram.dispatch((gridPointHorizontal + 7) / 8,
                                       (gridPointVertical + 7) / 8, 1);
    deformPageGeometryProgram.barrier(GL_SHADER_STORAGE_BARRIER_BIT |
                                      GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

    vao.bind();
    deformedPageVertexBuffer.bind_as(GL_ARRAY_BUFFER);
    pageIndexBuffer.bind_as(GL_ELEMENT_ARRAY_BUFFER);
    vao.link_attribute(deformedPageVertexBuffer, 0, 4, GL_FLOAT,
                       sizeof(vert_data), 0);
    vao.link_attribute(deformedPageVertexBuffer, 1, 4, GL_FLOAT,
                       sizeof(vert_data),
                       (void *)(offsetof(vert_data, texCoord)));
    shader.use();
    shader.set_float("gOffsetX", 0);
    shader.set_mat4("gVP", vp);
    shader.set_vec3("gEyeProtectionColor",
                    eyeProtection ? eyeProtectionColor : math::vector3::Ones());
    if (turnToNextPage) {
      shader.set_bool("gColored",
                      getTextureFromPool(leadingPageIdx + 1).get_format() !=
                          GL_RED);
      shader.set_bool(
          "gBackColored",
          leadingPageIdx + 2 >= pageCount + (padAfterFirstPage ? 1 : 0)
              ? whiteTexture.get_format() != GL_RED
              : getTextureFromPool(leadingPageIdx + 2).get_format() != GL_RED);
      shader.set_texture2d(
          "gPageTex", getTextureFromPool(leadingPageIdx + 1).get_handle(), 0);
      shader.set_texture2d(
          "gBackPageTex",
          leadingPageIdx + 2 >= pageCount + (padAfterFirstPage ? 1 : 0)
              ? whiteTexture.get_handle()
              : getTextureFromPool(leadingPageIdx + 2).get_handle(),
          1);
    } else {
      shader.set_bool("gColored",
                      leadingPageIdx >= pageCount + (padAfterFirstPage ? 1 : 0)
                          ? whiteTexture.get_handle() != GL_RED
                          : getTextureFromPool(leadingPageIdx).get_format() !=
                                GL_RED);
      shader.set_bool("gBackColored",
                      getTextureFromPool(leadingPageIdx - 1).get_format() !=
                          GL_RED);
      shader.set_texture2d(
          "gPageTex",
          leadingPageIdx >= pageCount + (padAfterFirstPage ? 1 : 0)
              ? whiteTexture.get_handle()
              : getTextureFromPool(leadingPageIdx).get_handle(),
          0);
      shader.set_texture2d("gBackPageTex",
                           getTextureFromPool(leadingPageIdx - 1).get_handle(),
                           1);
    }
    glDrawElements(GL_TRIANGLES,
                   (gridPointVertical - 1) * (gridPointHorizontal - 1) * 12,
                   GL_UNSIGNED_INT, 0);
    vao.unbind();
  }
}

void manga_viewer::scene_logic() {
  auto wndSize = toolkit::opengl::g_instance.get_window_size();
  float aspect = wndSize.x() / wndSize.y();
  auto scrollOffsets = toolkit::opengl::g_instance.get_scroll_offsets();
  mouseCurrentPos = toolkit::opengl::g_instance.get_mouse_position();
  bool mouseMidBtnPressed = toolkit::opengl::g_instance.is_mouse_button_pressed(
      GLFW_MOUSE_BUTTON_MIDDLE);
  bool mouseLeftBtnPressed =
      toolkit::opengl::g_instance.is_mouse_button_pressed(
          GLFW_MOUSE_BUTTON_LEFT);

  // reset camera back to default
  if (toolkit::opengl::g_instance.is_key_triggered(GLFW_KEY_R)) {
    logger->info("Reset camera parameters");
    cameraPos << 0.0f, 0.5f, 1.0f;
    cameraHalfRangeY = defaultCameraHalfRangeY;
  }

  // handle page translation
  if (mouseMidBtnPressed || mouseLeftBtnPressed) {
    // move around the camera
    if (mouseFirstMove) {
      mouseLastPos = mouseCurrentPos;
      mouseFirstMove = false;
    }

    // mid button pressed, left button not pressed, move camera position
    if (mouseMidBtnPressed && !mouseLeftBtnPressed) {
      // from screen space delta to camera space delta
      math::vector3 cameraSpaceDelta;
      cameraSpaceDelta << 2 * cameraHalfRangeY / wndSize.y() *
                              (mouseCurrentPos - mouseLastPos),
          0.0f;
      cameraSpaceDelta.x() *= -1;
      cameraPos += cameraSpaceDelta;
    }

    // mid button not pressed, left button pressed, flip page
    if (!mouseMidBtnPressed && mouseLeftBtnPressed) {
      math::vector2 mouseMoveDelta = mouseCurrentPos - mouseLastPos;
      math::vector2 mouseMoveDir = mouseMoveDelta.normalized();
      math::vector2 perpDir{mouseMoveDir.y(), -mouseMoveDir.x()};
      perpDir.normalize();
      float mouseMoveLength = mouseMoveDir.norm();
      // project the world position for cursor
      float convertRatio = 2 * cameraHalfRangeY / wndSize.y();
      float cursorX = cameraPos.x() +
                      (mouseCurrentPos.x() - 0.5 * wndSize.x()) * convertRatio;
      float cursorY = cameraPos.y() -
                      (mouseCurrentPos.y() - 0.5 * wndSize.y()) * convertRatio;
    }

    mouseLastPos = mouseCurrentPos;
  } else {
    mouseFirstMove = true;
  }

  // render the page geometry at different scale
  cameraHalfRangeY -= deltaTime * scrollOffsets.y();
  cameraHalfRangeY = std::clamp(cameraHalfRangeY, 0.1f, 3.0f);

  // update vp matrix for scene camera
  vp = math::ortho(-cameraHalfRangeY * aspect, cameraHalfRangeY * aspect,
                   cameraHalfRangeY, -cameraHalfRangeY, 1e-3f, 1e2f) *
       math::lookat(cameraPos, cameraPos - math::world_forward, math::world_up);

  float pageHeight = 1.0f;
  float pageWidth = first_page_width_div_height;
  if (!autoTurnPage && !(leadingPageIdx == -1 && pageFlowRTL) &&
      !(leadingPageIdx >= pageCount.load() - 1 && !pageFlowRTL) &&
      (toolkit::opengl::g_instance.is_key_triggered(GLFW_KEY_LEFT))) {
    pageFromRightToLeft = false;
    autoTurnPage = true;
    foldB = -1;
    foldK = foldB / pageWidth;
  }
  if (!autoTurnPage &&
      !(leadingPageIdx >= pageCount.load() - 1 && pageFlowRTL) &&
      !(leadingPageIdx == -1 && !pageFlowRTL) &&
      (toolkit::opengl::g_instance.is_key_triggered(GLFW_KEY_RIGHT))) {
    pageFromRightToLeft = true;
    autoTurnPage = true;
    foldB = -1;
    foldK = -foldB / pageWidth;
  }
}

void manga_viewer::main_loop() {
  glClearColor(backgroundColor.x(), backgroundColor.y(), backgroundColor.z(),
               1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, toolkit::opengl::g_instance.wnd_width,
             toolkit::opengl::g_instance.wnd_height);

  if (bookLoaded && !isLoadingBook) {
    if (!openSwitchPageWnd)
      scene_logic();
    draw_book();
  } else {
    // these variables should always gets reseted when not in use
    leadingPageIdx = -1;
    autoTurnPage = false;
    padAfterFirstPage = false;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  draw_gui();

  ImGui::EndFrame();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(toolkit::opengl::g_instance.window);
  deltaTime = (float)timer.elapse_s();
  timer.reset();
}

void manga_viewer::draw_gui() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::MenuItem("导入")) {
      std::string filepath;
      if (toolkit::open_file_dialog("选择一个 .pdf 或者 .epub 文件导入",
                                    {"*.PDF", "*.EPUB", "*.pdf", "*.epub"},
                                    "*.pdf, *.epub", filepath)) {
        std::thread t(&manga_viewer::on_load_file, this, filepath);
        t.detach();
      }
    }
    ImGui::SetItemTooltip("从 .pdf 或者 .epub 文件导入电子书");
    if (ImGui::BeginMenu("设置")) {
      ImGui::SeparatorText("导入选项");
      ImGui::Text(toolkit::str_format(
                      "dpi %d, (%d,%d,%d), %.3f MB", dpi, page_width.load(),
                      page_height.load(), page_channles.load(),
                      page_width.load() * page_height.load() *
                          page_channles.load() / (float)(1024 * 1024))
                      .c_str());
      static std::vector<std::string> dpi_names{"300", "216", "144", "72",
                                                "36"};
      gui::combo_default(
          "DPI", dpi_index, dpi_names,
          [&](int current) {
            if (current != -1) {
              dpi = std::stoi(dpi_names[current]);
              logger->info("set dpi to {0}", dpi);
            }
          },
          "Default");
      ImGui::SetItemTooltip(
          "导入一本电子书时，如果第一页占用内存的大小超过 "
          "max_page_size_mb，会执行一个默认的缩放确保内存占用不至于过大。");

      ImGui::SeparatorText("翻页选项");
      ImGui::Checkbox("右开本", &pageFlowRTL);
      ImGui::SetItemTooltip(
          "大陆的书籍基本上都是右开本，即从左向右翻页阅读。日本的轻小说和漫画大"
          "部分是左开本，需要从右向左翻页。");
      ImGui::Checkbox("填充扉页", &padAfterFirstPage);
      ImGui::SetItemTooltip("在扉页之后填充一个白色的空白页，如果漫画的跨页没有"
                            "对齐，可以用这个选项手动对齐。");
      ImGui::SliderFloat("翻页速度", &autoTurnPageSpeed, 0.1f, 10.0f);
      ImGui::SetItemTooltip("仿真书翻页动画的播放速度。");

      ImGui::SeparatorText("渲染选项");
      gui::color_edit_3("背景颜色", backgroundColor);
      ImGui::Checkbox("护眼模式", &eyeProtection);
      if (!eyeProtection)
        ImGui::BeginDisabled();
      gui::color_edit_3("护眼颜色", eyeProtectionColor);
      if (!eyeProtection)
        ImGui::EndDisabled();
      ImGui::SeparatorText("IO 选项");
      if (ImGui::Button("保存设置", {-1, 30}))
        dump_preference();
      if (ImGui::Button("载入设置", {-1, 30}))
        load_preference();

      ImGui::EndMenu();
    }

    ImGui::SameLine(ImGui::GetWindowWidth() -
                    ImGui::CalcTextSize("000 / 000   ").x -
                    ImGui::GetStyle().ItemSpacing.x);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0, 1.0, 0.0, 1.0));
    if (ImGui::Button(toolkit::str_format(" %3d /%3d ", leadingPageIdx + 1,
                                          pageCount.load())
                          .c_str())) {
      if (bookLoaded.load()) {
        openSwitchPageWnd = true;
        logger->info("Open popup");
      }
    }
    ImGui::PopStyleColor(1);

    ImGui::EndMainMenuBar();
  }

  static int pageNumber = 0;
  if (bookLoaded.load() && openSwitchPageWnd) {
    ImGui::OpenPopup("跳转到对应页数##jumptopage");
    auto size = ImGui::GetContentRegionAvail();
    auto center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowSize({0.7f * size.x, -1.0f}, ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("跳转到对应页数##jumptopage", &openSwitchPageWnd,
                               ImGuiWindowFlags_NoResize |
                                   ImGuiWindowFlags_NoMove)) {
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
      ImGui::InputInt("##Page Number", &pageNumber);

      float spacing = ImGui::GetStyle().ItemSpacing.x;
      float button_width = (ImGui::GetContentRegionAvail().x - spacing) * 0.5f;
      if (ImGui::Button("确认", ImVec2(button_width, 0))) {
        pageNumber = std::clamp(pageNumber, 1, pageCount.load());
        pageNumber -= 1;
        leadingPageIdx = pageNumber % 2 == 0 ? pageNumber - 1 : pageNumber;

        openSwitchPageWnd = false;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine(0, spacing);
      if (ImGui::Button("取消", ImVec2(button_width, 0))) {
        openSwitchPageWnd = false;
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
  } else {
    pageNumber = 0;
  }

  if (!bookLoaded && isLoadingBook) {
    ImGui::OpenPopup("导入中...##modalwindow");
    auto size = ImGui::GetContentRegionAvail();
    auto center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowSize({0.7f * size.x, -1.0f}, ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("导入中...##modalwindow", nullptr,
                               ImGuiWindowFlags_NoResize |
                                   ImGuiWindowFlags_NoMove)) {
      ImGui::ProgressBar(loadingProgress.load());
      ImGui::EndPopup();
    }
  }
}

std::string get_current_time_str() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
#ifdef _WIN32
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif

  std::stringstream ss;
  ss << std::setfill('0') << local_time.tm_year + 1900 << "-" << std::setw(2)
     << local_time.tm_mon + 1 << "-" << std::setw(2) << local_time.tm_mday
     << "-" << std::setw(2) << local_time.tm_hour << "-" << std::setw(2)
     << local_time.tm_min << "-" << std::setw(2) << local_time.tm_sec;
  return ss.str();
}

manga_viewer::manga_viewer() {
  toolkit::opengl::g_instance.init();
  ImGui::GetIO().IniFilename = nullptr;

  if (is_debug) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_pattern("[%H:%M:%S][%^%l%$] %v");
    logger = std::make_shared<spdlog::logger>("debug_logger", console_sink);
  } else {
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        toolkit::str_format("log/%s.log", get_current_time_str().c_str()));
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S][%l] %v");
    logger = std::make_shared<spdlog::logger>("release_logger", file_sink);
  }

  // enable vsync to save batery
  glfwSwapInterval(1);

  // prepare page geometry
  resizePageGrid();

  glEnable(GL_CULL_FACE);

  load_preference();

  resetTexturePool(8);

  ctx = fz_new_context(NULL, NULL, FZ_STORE_UNLIMITED);
  if (!ctx) {
    logger->error("Failed to create MuPDF context");
    return;
  }

  fz_try(ctx) fz_register_document_handlers(ctx);
  fz_catch(ctx) {
    logger->error("Cannot register document handlers");
    fz_drop_context(ctx);
    return;
  }
}
manga_viewer::~manga_viewer() {
  toolkit::opengl::g_instance.shutdown();

  if (doc)
    fz_drop_document(ctx, doc);
  if (ctx)
    fz_drop_context(ctx);
}

void manga_viewer::run() {
  toolkit::opengl::g_instance.run([&]() { main_loop(); });
}
