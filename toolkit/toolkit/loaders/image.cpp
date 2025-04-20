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
  filepath = path;
  stbi_image_free(_data);
  return true;
}

bool image::try_load_gray_scale(std::string path, int step_x, int step_y,
                                bool flipy) {
  stbi_set_flip_vertically_on_load(flipy);
  unsigned char *_data =
      stbi_load(path.c_str(), &width, &height, &nchannels, 0);
  if (!_data) {
    stbi_image_free(_data);
    return false;
  }
  bool colored = false;
  if (nchannels > 1) {
    for (int y = 0; y < height; y += step_y) {
      for (int x = 0; x < width; x += step_x) {
        unsigned char *pix = _data + y * width * nchannels + nchannels * x;
        unsigned char r = pix[0];
        unsigned char g = pix[1];
        if (std::abs(r - g) > 1 ||
            (nchannels > 2 && std::abs(pix[2] - g) > 1)) {
          colored = true;
          goto init_data;
        }
      }
    }
  }
init_data:
  if (colored) {
    data =
        std::vector<unsigned char>(_data, _data + width * height * nchannels);
  } else {
    data.resize(width * height);
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++)
        data[y * width + x] = _data[y * width * nchannels + x * nchannels];
    }
    nchannels = 1;
  }
  filepath = path;
  stbi_image_free(_data);
  return true;
}

void image::try_set_data_gray_scale(int w, int h, int n, unsigned char *d,
                                    int step_x, int step_y) {
  width = w;
  height = h;
  // optimize storage if possible
  bool colored = false;
  if (n > 1) {
    for (int y = 0; y < h; y += step_y) {
      for (int x = 0; x < w; x += step_x) {
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
    nchannels = n;
    data = std::vector<unsigned char>(d, d + w * h * n);
  } else {
    nchannels = 1;
    data.resize(w * h);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++)
        data[y * w + x] = d[y * w * n + x * n];
    }
  }
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

}; // namespace toolkit::assets