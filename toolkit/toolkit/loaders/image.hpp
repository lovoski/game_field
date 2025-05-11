#pragma once

#include "toolkit/math.hpp"
#include "toolkit/reflect.hpp"

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
  std::string filepath;

  void resize(int w, int h, int c);

  void try_set_data_gray_scale(int w, int h, int c, unsigned char *d,
                               int step_x = 10, int step_y = 10);

  const math::vector4 pixel(int x, int y);
  unsigned char &pixel(int x, int y, int channel);
  /**
   * Load an image of format .png, .jpeg, .bmp, .tga, flip vertically if `flipy`
   * set as `true`
   */
  bool load(std::string path, bool flipy = false);
  bool try_load_gray_scale(std::string path, int step_x = 10, int step_y = 10,
                           bool flipy = false);
  /**
   * Save the image into format .png, flip vertically if `flipy`
   * set as `true`
   */
  bool save_png(std::string path, bool flipy = false);
};
REFLECT(image, data, width, height, nchannels, filepath)

}; // namespace toolkit::assets