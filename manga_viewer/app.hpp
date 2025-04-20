#pragma once

#include "toolkit/math.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <mupdf/fitz.h>

class thread_pool {
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  std::mutex queue_mutex;
  std::condition_variable condition;
  bool stop = false;

public:
  thread_pool(size_t threads = 8) {
    for (size_t i = 0; i < threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty())
              return;
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  template <class F> void enqueue(F &&f) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
  }

  ~thread_pool() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      stop = true;
    }
    condition.notify_all();
    for (auto &worker : workers)
      worker.join();
  }
};

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

class manga_viewer {
public:
  manga_viewer();
  ~manga_viewer();

  void run();

private:
  void main_loop();

  void draw_gui();
  void draw_book();

  void on_load_file(std::string filepath);

  void scene_logic();

  std::shared_ptr<spdlog::logger> logger = nullptr;

  int dpi = 300, dpi_index = -1;
  float max_page_size_mb = 8.0f;

  const std::string preferencePath = "./preference.json";
  std::string cacheDirPath, activeFilePath;
  void dump_preference();
  void load_preference();

  fz_context *ctx = nullptr;
  fz_document *doc = nullptr;

  std::atomic<bool> bookLoaded{false}, isLoadingBook{false};
  std::atomic<int> pageCount{0}, pageLoaded{0};
  std::atomic<float> loadingProgress{0.0f};

  int texturePoolIdx = 0;
  thread_safe_vector<int> texturePoolPageIdxData;
  thread_safe_vector<toolkit::opengl::texture> texturePoolData;
  thread_safe_vector<std::string> bookPageFilePathes;
  std::atomic<float> first_page_width_div_height;
  void resetTexturePool(int limit);
  void loadPageCacheFromFile(int pageIdx, toolkit::assets::image &result);
  const toolkit::opengl::texture &getTextureFromPool(int pageIdx);
  thread_safe_vector<std::pair<int, toolkit::assets::image>> highResImageQueue;
  std::mutex highResPageLoadingLock;
  thread_pool tpool;
  void loadHighResPage(int pageIdx);
  void applyHighResQueue();

  float gridCellSizeX, gridCellSizeY;

  std::atomic<int> page_width{0}, page_height{0}, page_channles{0};

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

#ifdef _DEBUG
  bool is_debug = true;
#elif !defined(NDEBUG)
  bool is_debug = true;
#else
  bool is_debug = false;
#endif

  REFLECT_PRIVATE(manga_viewer)
};
REFLECT(manga_viewer, autoTurnPageSpeed, eyeProtection, eyeProtectionColor,
        backgroundColor, pageFlowRTL)
