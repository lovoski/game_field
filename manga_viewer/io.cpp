#include "app.hpp"
#include "toolkit/loaders/image.hpp"
#include <filesystem>
#include <thread>

std::string uint32_to_binary_str(std::uint32_t n) {
  std::string result = "";
  for (int i = 0; i < 32; i++) {
    if (((n >> i) & 1) == 0) {
      result = '0' + result;
    } else {
      result = '1' + result;
    }
  }
  return result;
}

std::string uint32_to_str(std::uint32_t n, int limit = 6) {
  std::string result = "";
  std::uint32_t r = n;
  for (int i = 0; i < limit; i++) {
    int digit = (n / (int)std::pow(10, i)) % 10;
    result = (char)(digit + '0') + result;
  }
  return result;
}

void box_filter_scale(const uint8_t *input, int iw, int ih, int channels,
                      uint8_t *output, int tw, int th) {
  for (int ty = 0; ty < th; ++ty) {
    int sy_start = ty * ih / th;
    int sy_end = (ty + 1) * ih / th;
    for (int tx = 0; tx < tw; ++tx) {
      int sx_start = tx * iw / tw;
      int sx_end = (tx + 1) * iw / tw;
      for (int c = 0; c < channels; ++c) {
        int sum = 0;
        int count = 0;
        for (int sy = sy_start; sy < sy_end; ++sy) {
          for (int sx = sx_start; sx < sx_end; ++sx) {
            sum += input[(sy * iw + sx) * channels + c];
            ++count;
          }
        }
        output[(ty * tw + tx) * channels + c] =
            static_cast<uint8_t>(sum / count);
      }
    }
  }
}

void scaleImage(std::string source, std::string output,
                unsigned int fixedHeight = 100) {
  toolkit::assets::image sImg, tImg;
  sImg.load(source);
  if (sImg.height <= fixedHeight) {
    // save the original image if it's already smaller than the desired low res
    sImg.save_png(output);
    return;
  }
  tImg.height = fixedHeight;
  tImg.width = static_cast<int>((static_cast<float>(sImg.width) / sImg.height) *
                                fixedHeight);
  tImg.nchannels = sImg.nchannels;
  tImg.data.resize(tImg.height * tImg.width * tImg.nchannels);
  box_filter_scale(sImg.data.data(), sImg.width, sImg.height, sImg.nchannels,
                   tImg.data.data(), tImg.width, tImg.height);
  tImg.save_png(output);
}

void manga_viewer::on_load_file(std::string filepath) {
  bookLoaded = false;
  isLoadingBook = true;

  if (doc)
    fz_drop_document(ctx, doc);
  fz_try(ctx) doc = fz_open_document(ctx, filepath.c_str());
  fz_catch(ctx) {
    logger->error("Failed to open document: {0}", fz_caught_message(ctx));
    fz_drop_context(ctx);
    bookLoaded = false;
    isLoadingBook = false;
    return;
  }
  pageCount.store(fz_count_pages(ctx, doc));

  // record current active file
  activeFilePath = filepath;

  // update the window title
  glfwSetWindowTitle(toolkit::opengl::g_instance.window, filepath.c_str());

  // clear cached pages
  texturePoolPageIdxData.on([&](std::vector<int> &tppid) {
    for (auto &idx : tppid)
      idx = -1;
  });

  // record the size of the first image
  fz_page *page = nullptr;
  fz_pixmap *pixmap = nullptr;
  fz_matrix first_page_transform = fz_scale(dpi / 72.0f, dpi / 72.0f);
  fz_try(ctx) {
    page = fz_load_page(ctx, doc, 0);
    fz_bound_page(ctx, page);

    pixmap = fz_new_pixmap_from_page_number(ctx, doc, 0, first_page_transform,
                                            fz_device_rgb(ctx), 0);

    page_width.store(pixmap->w);
    page_height.store(pixmap->h);
    page_channles.store(pixmap->n);

    float size_mb = pixmap->w * pixmap->h * pixmap->n / (float)(1024 * 1024);
    if (size_mb > max_page_size_mb) {
      // adjust dpi to suit settings
      float scale = std::sqrt(max_page_size_mb / size_mb);
      dpi = dpi * scale;
      dpi_index = -1;
    }

    first_page_width_div_height.store(pixmap->w / (float)pixmap->h);
  }
  fz_always(ctx) {
    fz_drop_pixmap(ctx, pixmap);
    fz_drop_page(ctx, page);
  }
  fz_catch(ctx) {}

  logger->info("Loading a book with {0} pages from {1}", pageCount.load(),
               filepath);
  bookLoaded = true;
  isLoadingBook = false;
}

void manga_viewer::resetTexturePool(int limit) {
  limit = std::clamp(limit, 1, 255);
  texturePoolIdx = 0;
  texturePoolData.on([&](std::vector<toolkit::opengl::texture> &texData) {
    for (auto &tex : texData)
      tex.del();
    texData.clear();
    texData.resize(limit);
    for (auto &tex : texData)
      tex.create();
  });
  texturePoolPageIdxData.on(
      [&](std::vector<int> &tppid) { tppid.resize(limit, -1); });
}

const toolkit::opengl::texture &manga_viewer::getTextureFromPool(int pageIdx) {
  // pageIdx can't be negative
  pageIdx = std::max(0, pageIdx);
  if (!bookLoaded.load() || (padAfterFirstPage && pageIdx == 1))
    return whiteTexture;
  pageIdx = std::min(pageIdx, pageCount.load() + (padAfterFirstPage ? 0 : -1));
  if (padAfterFirstPage && pageIdx > 1)
    pageIdx -= 1;
  int matchCachedPageIdx = -1;
  texturePoolPageIdxData.on([&](std::vector<int> &tppid) {
    for (int i = 0; i < tppid.size(); i++) {
      if (tppid[i] == pageIdx) {
        matchCachedPageIdx = i;
        break;
      }
    }
  });
  if (matchCachedPageIdx != -1) {
    // find matching page cache
    return texturePoolData.get(matchCachedPageIdx);
  }
  // no cached page texture found, reload from file
  texturePoolIdx = (texturePoolIdx + 1) % texturePoolData.size();
  toolkit::assets::image pageCache;
  // load low res image or white texture
  loadPageCacheFromFile(pageIdx, pageCache);
  texturePoolData.on([&](std::vector<toolkit::opengl::texture> &texData) {
    texData[texturePoolIdx].set_data_from_image(pageCache);
    texData[texturePoolIdx].set_parameters(
        {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
         {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
         {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
         {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  });
  texturePoolPageIdxData.on(
      [&](std::vector<int> &tppid) { tppid[texturePoolIdx] = pageIdx; });
  // start loading the high res image after everything's setup
  std::thread t(&manga_viewer::loadHighResPage, this, pageIdx);
  t.detach();

  return texturePoolData.get(texturePoolIdx);
}

void manga_viewer::loadPageCacheFromFile(int pageIdx,
                                         toolkit::assets::image &result) {
  std::lock_guard<std::mutex> lock(docLoadingLock);
  // pageIdx is ensured to be in range
  static unsigned char errorPixel[3] = {247, 0, 247};
  static unsigned char whitePixel[3] = {255, 255, 255};
  fz_pixmap *pixmap = nullptr;
  fz_matrix transform = fz_scale(0.05f, 0.05f);
  // fz_matrix transform = fz_scale(1, 1);
  fz_try(ctx) {
    pixmap = fz_new_pixmap_from_page_number(ctx, doc, pageIdx, transform,
                                            fz_device_rgb(ctx), 0);

    page_width.store(pixmap->w);
    page_height.store(pixmap->h);
    page_channles.store(pixmap->n);

    result.try_set_data_gray_scale(pixmap->w, pixmap->h, pixmap->n,
                                   pixmap->samples);
  }
  fz_always(ctx) { fz_drop_pixmap(ctx, pixmap); }
  fz_catch(ctx) {
    logger->error("Error loading high resolution page {0}, {1}", pageIdx,
                  fz_caught_message(ctx));
    result.try_set_data_gray_scale(1, 1, 3, errorPixel);
  }
}

void manga_viewer::loadHighResPage(int pageIdx) {
  std::lock_guard<std::mutex> lock(docLoadingLock);
  unsigned char errorPixel[3] = {247, 0, 247};
  static unsigned char whitePixel[3] = {255, 255, 255};
  toolkit::assets::image pageCache;

  // spdlog::warn("load page {0}", pageIdx);

  if (doc == nullptr) {
    logger->error("Can't load from nullptr document at loadHighResPage");
    pageCache.try_set_data_gray_scale(1, 1, 3, errorPixel);
    highResImageQueue.on(
        [&](std::vector<std::pair<int, toolkit::assets::image>> &hriq) {
          hriq.push_back(std::make_pair(pageIdx, pageCache));
        });
    return;
  }
  // fz_context *tmp_ctx = fz_clone_context(ctx);
  fz_pixmap *pixmap = nullptr;
  fz_matrix transform = fz_scale(dpi / 72.0f, dpi / 72.0f);
  // fz_matrix transform = fz_scale(1, 1);
  fz_try(ctx) {
    pixmap = fz_new_pixmap_from_page_number(ctx, doc, pageIdx, transform,
                                            fz_device_rgb(ctx), 0);

    page_width.store(pixmap->w);
    page_height.store(pixmap->h);
    page_channles.store(pixmap->n);

    pageCache.try_set_data_gray_scale(pixmap->w, pixmap->h, pixmap->n,
                                      pixmap->samples);
  }
  fz_always(ctx) { fz_drop_pixmap(ctx, pixmap); }
  fz_catch(ctx) {
    logger->error("Error loading high resolution page {0}, {1}", pageIdx,
                  fz_caught_message(ctx));
    pageCache.try_set_data_gray_scale(1, 1, 3, errorPixel);
  }
  // fz_drop_context(tmp_ctx);

  // check for page widht height ratio
  float ref_ratio = first_page_width_div_height.load();
  float wh_ratio = pageCache.width / (float)pageCache.height,
        hw_ratio = pageCache.height / (float)pageCache.width;
  if (abs(wh_ratio-ref_ratio) >= abs(hw_ratio-ref_ratio)) {
    logger->info("try rotate page {0} to fit width/height ratio", pageIdx);
    // try rotate the page
    toolkit::assets::image img;
    img.resize(pageCache.height, pageCache.width, pageCache.nchannels);
    for (int x = 0; x < pageCache.width; x++)
      for (int y = 0; y < pageCache.height; y++)
        for (int c = 0; c < pageCache.nchannels; c++)
          img.pixel(pageCache.height-y-1,x,c) = pageCache.pixel(x,y,c);
    img.filepath = pageCache.filepath;
    pageCache = img;
  }

  logger->info("load high-res page {0}, ({1},{2},{3}), {4} MB", pageIdx,
               pageCache.width, pageCache.height, pageCache.nchannels,
               pageCache.width * pageCache.height * pageCache.nchannels /
                   (float)(1024 * 1024));

  highResImageQueue.on(
      [&](std::vector<std::pair<int, toolkit::assets::image>> &hriq) {
        hriq.emplace_back(std::make_pair(pageIdx, pageCache));
      });

  // spdlog::warn("page {0} loaded", pageIdx);
}

void manga_viewer::applyHighResQueue() {
  highResImageQueue.on(
      [&](std::vector<std::pair<int, toolkit::assets::image>> &hriq) {
        // apply queue data to the actual texture
        std::vector<int> matchedIndices;
        texturePoolPageIdxData.on([&](std::vector<int> &tppid) {
          matchedIndices.resize(hriq.size(), -1);
          for (int i = 0; i < hriq.size(); i++) {
            for (int j = 0; j < tppid.size(); j++) {
              if (hriq[i].first == tppid[j])
                matchedIndices[i] = j;
            }
          }
        });
        texturePoolData.on([&](std::vector<toolkit::opengl::texture> &texData) {
          for (int i = 0; i < matchedIndices.size(); i++) {
            if (matchedIndices[i] == -1)
              continue;
            auto &pageData = hriq[i].second;
            texData[matchedIndices[i]].set_data_from_image(pageData);
            texData[matchedIndices[i]].set_parameters(
                {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                 {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                 {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                 {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
          }
        });
        hriq.clear();
      });
}

void manga_viewer::dump_preference() {
  try {
    std::ofstream output(preferencePath);
    nlohmann::json data = *this;
    output << data.dump(2) << std::endl;
    output.close();
    logger->info("Dump current settings to user preference file.");
  } catch (std::exception e) {
    logger->error("Failed to dump user preference, {0}", e.what());
  }
}
void manga_viewer::load_preference() {
  if (!std::filesystem::exists(preferencePath)) {
    logger->warn(
        "User preference don't exists, create one with current settings.");
    dump_preference();
    return;
  }
  try {
    std::ifstream input(preferencePath);
    auto data = nlohmann::json::parse(
        std::string((std::istreambuf_iterator<char>(input)),
                    std::istreambuf_iterator<char>()));
    from_json(data, *this);
    logger->info("Load user preference file to current settings.");
  } catch (std::exception e) {
    logger->error("Failed to load user preference, {0}", e.what());
  }
}
