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

  cacheDirPath = toolkit::join_path(
      ".", "book_cache", std::to_string(toolkit::string_hash(filepath)));
  if (!std::filesystem::exists(cacheDirPath)) {
    // load book from source file, cache in caheDirPath
    toolkit::mkdir(cacheDirPath);
    toolkit::mkdir(toolkit::join_path(cacheDirPath, "low_res"));
    pageCount.store(fz_count_pages(ctx, doc));
    // render book into .png images in cache dir
    fz_matrix transform = fz_scale(0.1, 0.1);
    for (int pageIdx = 0; pageIdx < pageCount; pageIdx++) {
      fz_page *page = nullptr;
      fz_pixmap *pixmap = nullptr;

      fz_try(ctx) {
        page = fz_load_page(ctx, doc, pageIdx);
        fz_bound_page(ctx, page);

        pixmap = fz_new_pixmap_from_page_number(ctx, doc, pageIdx, transform,
                                                fz_device_rgb(ctx), 0);

        std::string imageFilename =
            toolkit::str_format("%s.png", uint32_to_str(pageIdx).c_str());
        std::string cacheImageFilepath =
            toolkit::join_path(cacheDirPath, "low_res", imageFilename);
        fz_save_pixmap_as_png(ctx, pixmap, cacheImageFilepath.c_str());

        loadingProgress = (pageIdx + 1) / (float)pageCount;
      }
      fz_always(ctx) {
        fz_drop_pixmap(ctx, pixmap);
        fz_drop_page(ctx, page);
      }
      fz_catch(ctx) {
        logger->error("Failed to load page {0} from file {1}", pageIdx,
                      filepath);
        bookLoaded = false;
        isLoadingBook = false;
        return;
      }
    }
  }

  // record current active file
  activeFilePath = filepath;

  // update the window title
  glfwSetWindowTitle(toolkit::opengl::g_instance.window, filepath.c_str());

  // clear cached pages
  texturePoolPageIdxData.on([&](std::vector<int> &tppid) {
    for (auto &idx : tppid)
      idx = -1;
  });
  // record cached pathes
  bookPageFilePathes.on([&](std::vector<std::string> &filepathData) {
    filepathData.clear();
    // list the files within this directory ended with .png
    for (auto entry : std::filesystem::directory_iterator(
             toolkit::join_path(cacheDirPath, "low_res"))) {
      if (entry.is_regular_file()) {
        auto extension = entry.path().extension().string();
        if (extension == ".png") {
          filepathData.push_back(entry.path().string());
        }
      }
    }
    pageCount.store(filepathData.size());
    // record the size of the first image
    fz_page *page = nullptr;
    fz_pixmap *pixmap = nullptr;
    fz_matrix first_page_transform = fz_scale(dpi / 72.0f, dpi / 72.0f);
    fz_try(ctx) {
      page = fz_load_page(ctx, doc, 0);
      fz_bound_page(ctx, page);

      pixmap = fz_new_pixmap_from_page_number(ctx, doc, 0, first_page_transform,
                                              fz_device_rgb(ctx), 0);

      first_page_width_div_height.store(pixmap->w / (float)pixmap->h);
    }
    fz_always(ctx) {
      fz_drop_pixmap(ctx, pixmap);
      fz_drop_page(ctx, page);
    }
    fz_catch(ctx) {}
  });

  logger->info("Loading a book with {0} pages", pageCount.load());
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
  page_cache pageCache;
  // load low res image or white texture
  loadPageCacheFromFile(pageIdx, pageCache);
  int nChannels = pageCache.channels;
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
  }
  texturePoolData.on([&](std::vector<toolkit::opengl::texture> &texData) {
    texData[texturePoolIdx].set_data(pageCache.width, pageCache.height, iformat,
                                     format, GL_UNSIGNED_BYTE,
                                     pageCache.data.data());
    texData[texturePoolIdx].set_parameters(
        {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
         {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
         {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
         {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  });
  texturePoolPageIdxData.on(
      [&](std::vector<int> &tppid) { tppid[texturePoolIdx] = pageIdx; });
  // start loading the high res image after everything's setup
  std::thread t(&manga_viewer::loadHighResPage, this, activeFilePath, pageIdx);
  t.detach();

  return texturePoolData.get(texturePoolIdx);
}

void manga_viewer::loadPageCacheFromFile(int pageIdx, page_cache &result) {
  // pageIdx is ensured to be in range
  static unsigned char errorPixel[3] = {247, 0, 247};
  static unsigned char whitePixel[3] = {255, 255, 255};
  bookPageFilePathes.on([&](std::vector<std::string> &pagePathes) {
    if (pageIdx >= pagePathes.size()) {
      logger->error("Something wrong with the pageIdx {0}, pageCount {1}, "
                    "pagePathes.size() {2}, returns error "
                    "page by default",
                    pageIdx, pageCount.load(), pagePathes.size());
      result.set_data(1, 1, 3, errorPixel);
      return;
    }
    // first try loading the image in low res if possible
    auto cache_filepath = std::filesystem::path(pagePathes[pageIdx]);
    if (std::filesystem::exists(cache_filepath)) {
      // there's a low res version of the file, load it first
      fz_image *image = nullptr;
      fz_pixmap *pixmap = nullptr;
      fz_try(ctx) {
        image = fz_new_image_from_file(ctx, cache_filepath.string().c_str());
        // Create a pixmap from the image
        pixmap = fz_get_pixmap_from_image(ctx, image, nullptr, nullptr, nullptr,
                                          nullptr);
        result.set_data(pixmap->w, pixmap->h, pixmap->n, pixmap->samples);
      }
      fz_always(ctx) {
        fz_drop_pixmap(ctx, pixmap);
        fz_drop_image(ctx, image);
      }
      fz_catch(ctx) {
        logger->error("Error loading page {0}, : {1}", pageIdx,
                      fz_caught_message(ctx));
        result.set_data(1, 1, 3, errorPixel);
      }
    } else {
      // no low res version found for this file, try return a blank image
      result.set_data(1, 1, 3, whitePixel);
    }
  });
}

void manga_viewer::loadHighResPage(std::string filepath, int pageIdx) {
  std::lock_guard<std::mutex> lock(highResPageLoadingLock);
  unsigned char errorPixel[3] = {247, 0, 247};
  static unsigned char whitePixel[3] = {255, 255, 255};
  page_cache pageCache;
  if (doc == nullptr) {
    logger->error("Can't load from nullptr document at loadHighResPage");
    pageCache.set_data(1, 1, 3, errorPixel);
    highResImageQueue.on([&](std::vector<std::pair<int, page_cache>> &hriq) {
      hriq.push_back(std::make_pair(pageIdx, pageCache));
    });
    return;
  }
  fz_pixmap *pixmap = nullptr;
  fz_matrix transform = fz_scale(dpi / 72.0f, dpi / 72.0f);
  fz_try(ctx) {
    pixmap = fz_new_pixmap_from_page_number(ctx, doc, pageIdx, transform,
                                            fz_device_rgb(ctx), 0);

    pageCache.set_data(pixmap->w, pixmap->h, pixmap->n, pixmap->samples);
  }
  fz_always(ctx) {
    fz_drop_pixmap(ctx, pixmap);
  }
  fz_catch(ctx) {
    logger->error("Error loading high resolution page {0}, {1}", pageIdx,
                  fz_caught_message(ctx));
    pageCache.set_data(1, 1, 3, errorPixel);
  }
  highResImageQueue.on([&](std::vector<std::pair<int, page_cache>> &hriq) {
    hriq.push_back(std::make_pair(pageIdx, pageCache));
  });
}

void manga_viewer::applyHighResQueue() {
  highResImageQueue.on([&](std::vector<std::pair<int, page_cache>> &hriq) {
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
        int nChannels = pageData.channels;
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
        }
        texData[matchedIndices[i]].set_data(pageData.width, pageData.height,
                                            iformat, format, GL_UNSIGNED_BYTE,
                                            pageData.data.data());
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

void page_cache::set_data(int w, int h, int n, unsigned char *d,
                          unsigned int stepX, unsigned int stepY) {
  width = w;
  height = h;
  // optimize storage if possible
  bool colored = false;
  if (n > 1) {
    for (int y = 0; y < h; y += stepY) {
      for (int x = 0; x < w; x += stepX) {
        unsigned char *pix = d + y * w * n + n * x;
        unsigned char r = pix[0];
        unsigned char g = pix[1];
        if (std::abs(r - g) > 1 || (n > 2 && std::abs(pix[2] - g) > 1)) {
          colored = true;
          goto init_data;
        }
      }
    }
  }
init_data:
  if (colored) {
    channels = n;
    data = std::vector<unsigned char>(d, d + w * h * n);
  } else {
    // logger->info("Optimize page to gray scale image");
    channels = 1;
    data.resize(w * h);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++)
        data[y * w + x] = d[y * w * n + x * n];
    }
  }
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
