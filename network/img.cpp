#include "mnist.hpp"
#include <toolkit/loaders/image.hpp>
#include <toolkit/opengl/base.hpp>

int main() {
  std::string filepath;
  MNIST model;
  int total_count = 0, acc_count = 0, current_count = 0;
  if (toolkit::open_file_dialog("Select MNIST dataset label annotation",
                                {"*.json"}, "*.json", filepath)) {
    std::ifstream input(filepath);
    auto data = nlohmann::json::parse(input);
    total_count = data.size();
    for (auto [key, val] : data.items()) {
      toolkit::assets::image img;
      int gt = val.get<int>();
      img.load(key);
      std::array<float, MNIST::width_ * MNIST::height_> img_data;
      for (int w = 0; w < img.width; w++)
        for (int h = 0; h < img.height; h++)
          img_data[w + h * img.width] = img.pixel(w, h, 0) / (float)255;
      auto pred = model.run(img_data);
      current_count++;
      if (gt == pred)
        acc_count++;
      printf("%d/%d\n", current_count, total_count);
    }
  }
  printf("accuracy: %.3f\n", acc_count / (float)total_count);
  return 0;
}