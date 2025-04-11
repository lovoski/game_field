#include "toolkit/loaders/image.hpp"
#include "toolkit/loaders/imp.hpp"

namespace toolkit::assets {

const math::vector4 image::pixel(int x, int y) {
  math::vector4 ret = math::vector4::Zero();
  for (int c = 0; c < nchannels; c++)
    ret[c] = (float)pixel(x, y, c) / 255;
  return ret;
}

void image::resize(int w, int h, int c) {
  width = w;
  height = h;
  nchannels = c;
  data.clear();
  data.resize(w * h * c);
}

// Clamp to range [min, max]
void clamp(int &value, int min, int max) {
  if (value < min)
    value = min;
  if (value > max)
    value = max;
}

// Flip image vertically
void flip_image_vertically(unsigned char *image, int width, int height,
                           int channels) {
  int row_size = width * channels; // Number of bytes in one row
  unsigned char *row_buffer =
      (unsigned char *)malloc(row_size); // Temporary buffer for swapping rows
  for (int y = 0; y < height / 2; y++) {
    unsigned char *top_row = image + y * row_size;
    unsigned char *bottom_row = image + (height - 1 - y) * row_size;
    // Swap top_row and bottom_row
    memcpy(row_buffer, top_row, row_size);
    memcpy(top_row, bottom_row, row_size);
    memcpy(bottom_row, row_buffer, row_size);
  }
  free(row_buffer); // Free the temporary buffer
}

bool image::load(std::string path, bool flip) {
  stbi_set_flip_vertically_on_load(flip);
  unsigned char *_data =
      stbi_load(path.c_str(), &width, &height, &nchannels, 0);
  if (!_data) {
    stbi_image_free(_data);
    return false;
  }
  data.resize(width * height * nchannels);
  std::memcpy(data.data(), _data, width * height * nchannels);
  imagePath = path;
  stbi_image_free(_data);
  return true;
}

bool image::save_png(std::string path, bool flip) {
  auto saveData = data;
  if (flip)
    flip_image_vertically(saveData.data(), width, height, nchannels);
  return stbi_write_png(path.c_str(), width, height, nchannels, saveData.data(),
                        width * nchannels);
}

unsigned char &image::pixel(int x, int y, int channel) {
  clamp(x, 0, width - 1);
  clamp(y, 0, height - 1);
  clamp(channel, 0, nchannels - 1);
  return data[(y * width + x) * nchannels + channel];
}

}; // namespace toolkit::Assets