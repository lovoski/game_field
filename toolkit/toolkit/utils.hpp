#pragma once

#include "toolkit/math.hpp"
#include <chrono>
#include <filesystem>

namespace toolkit {

std::string lower_case(std::string str);
std::string uppper_case(std::string str);
// Check if `base` has `pattern` as a continuous substring.
bool has_substr(std::string base, std::string pattern);
int string_hash(const std::string str);
std::string replace(std::string str, std::string pattern, std::string replace);
bool endswith(std::string target, std::string pattern);
std::string str_format(const char *format, ...);

inline std::filesystem::path
relpath(std::filesystem::path filepath,
        std::filesystem::path base = std::filesystem::current_path()) {
  return std::filesystem::relative(filepath, base);
}
inline std::filesystem::path abspath(std::filesystem::path filepath) {
  return std::filesystem::absolute(filepath);
}

void copy_file(std::filesystem::path src_filepath,
               std::filesystem::path dst_filepath);
void copy_dir(std::filesystem::path src_dirpath,
              std::filesystem::path dst_dirpath);

bool open_folder_dialog(std::string title, std::string &selectedFolder);

bool save_file_dialog(std::string title,
                      std::vector<const char *> filterPatterns,
                      std::string description, std::string &selectedFile);

bool open_file_dialog(std::string title,
                      std::vector<const char *> filterPatterns,
                      std::string description, std::string &selectedFile);

bool listdir(std::string dirpath, std::function<void(std::string)> f);

bool mkdir(std::string path);

template <typename... Paths> std::string join_path(Paths &&...paths) {
  std::filesystem::path result;
  (result /= ... /= std::filesystem::path(paths));
  return result.string();
}

class stopwatch {
public:
  stopwatch() { reset(); }

  ~stopwatch() {}

  // Resets the timer to the current time
  void reset() { start_time = std::chrono::high_resolution_clock::now(); }

  // Returns the elapsed time in milliseconds since the timer started or was
  // last reset
  double elapse_ms() const {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    return duration.count();
  }

  // Returns the elapsed time in seconds since the timer started or was last
  // reset
  double elapse_s() const {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    return duration.count();
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

}; // namespace toolkit