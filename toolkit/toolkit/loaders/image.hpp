#pragma once

#include "toolkit/math.hpp"

namespace toolkit::assets {

/**
 * Image data format:
 *
 * index = (y * width + x) * nchannels + channel
 *
 * (0, 0) --- (w, 0)
 * (0, h) --- (w, h)
 *
 * Mind that the parameter `path` for save and load can't contain chinese
 * characters
 */
struct image {
  std::vector<unsigned char> data;
  int width, height, nchannels;
  std::string imagePath;

  void resize(int w, int h, int c);

  const math::vector4 pixel(int x, int y);
  unsigned char &pixel(int x, int y, int channel);
  /**
   * Load an image of format .png, .jpeg, .bmp, .tga, flip vertically if `flipy`
   * set as `true`
   */
  bool load(std::string path, bool flipy = false);
  /**
   * Save the image into format .png, flip vertically if `flipy`
   * set as `true`
   */
  bool save_png(std::string path, bool flipy = false);
};

}; // namespace Common::Assets