#pragma once

#include "toolkit/math.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"

#include <atomic>
#include <mutex>
#include <thread>

#include <mupdf/fitz.h>

template <typename T> class thread_safe_vector {
public:
  thread_safe_vector() {}
  ~thread_safe_vector() {}

  void on(std::function<void(std::vector<T> &)> f) {
    std::lock_guard<std::mutex> lock(mtx);
    f(data);
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mtx);
    return data.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mtx);
    data.clear();
  }

  const T &get(std::size_t index) const {
    std::lock_guard<std::mutex> lock(mtx);
    return data[index];
  }

private:
  mutable std::mutex mtx;
  std::vector<T> data;
};

class page_cache {
public:
  page_cache() {}
  ~page_cache() {}
  void set_data(int w, int h, int n, unsigned char *d, unsigned int stepX = 10,
                unsigned int stepY = 10);
  int width, height, channels;
  std::vector<unsigned char> data;
};

class manga_viewer {
public:
  manga_viewer();
  ~manga_viewer();

  void run();

private:
  void mainLoop();

  void drawGUI();
  void drawBook();

  void loadSingleFile(std::string filepath);
  void loadFolderFile(std::string folderpath);

  void sceneLogic();

  int dpi = 300;

  const std::string preferencePath = "./pref.json";
  std::string cacheDirPath;
  void dumpPreference();
  void loadPreference();

  fz_context *ctx = nullptr;
  fz_document *doc = nullptr;
  std::atomic<bool> bookLoaded{false}, isLoadingBook{false};
  std::atomic<int> pageCount{0}, pageLoaded{0};
  std::atomic<float> loadingProgress{0.0f};

  int texturePoolIdx = 0;
  thread_safe_vector<int> texturePoolPageIdxData;
  thread_safe_vector<toolkit::opengl::texture> texturePoolData;
  thread_safe_vector<std::string> bookPageFilePathes;
  std::atomic<int> firstPageWidth, firstPageHeight;
  void resetTexturePool(int limit);
  std::string loadPageCacheFromFile(int pageIdx, page_cache &result);
  const toolkit::opengl::texture &getTextureFromPool(int pageIdx);
  thread_safe_vector<std::pair<int, page_cache>> highResImageQueue;
  std::mutex highResImageLoadingLock;
  void loadHighResImage(std::string filepath, int pageIdx);
  void applyHighResQueue();

  float gridCellSizeX, gridCellSizeY;

  float autoTurnPageSpeed = 1.0f;
  float foldK = 2, foldB = -1;
  float foldKInterpStart = 10, foldKInterpEnd = 100.0f;
  bool pageFromRightToLeft = true, autoTurnPage = false,
       manuallyTurningPage = false;
  bool pageFlowRTL = true;
  toolkit::opengl::texture whiteTexture;
  toolkit::opengl::compute_shader createPageGeometryProgram,
      deformPageGeometryProgram;
  toolkit::opengl::buffer pageIndexBuffer, pageVertexBuffer,
      deformedPageVertexBuffer;

  bool padAfterFirstPage = false;

  const float defaultCameraHalfRangeY = 0.6f;
  float cameraHalfRangeY = defaultCameraHalfRangeY;
  toolkit::math::vector3 cameraPos = toolkit::math::vector3(0, 0.5f, 1);
  toolkit::math::matrix4 vp = toolkit::math::matrix4::Identity();

  bool mouseFirstMove = true;
  toolkit::math::vector2 mouseLastPos, mouseCurrentPos;

  float deltaTime = 0.0f;
  toolkit::stopwatch timer;

  bool eyeProtection = false;
  toolkit::math::vector3 eyeProtectionColor{0.93, 0.90, 0.78};

  toolkit::math::vector3 backgroundColor{0.28, 0, 0.12};

  // ranging [-1, pageCount], with -1 being null, pageCount being white if
  // don't exists
  int leadingPageIdx = -1;

  unsigned int gridPointHorizontal = 50, gridPointVertical = 50;
  void resizePageGrid();
};