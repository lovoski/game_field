#include "toolkit/utils.hpp"
#include <cctype>
#include <cstdarg>
#include <tinyfiledialogs.h>

namespace fs = std::filesystem;

namespace toolkit {

std::string lower_case(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return str;
}
std::string uppper_case(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return str;
}
bool has_substr(std::string base, std::string pattern) {
  if (pattern.size() == 0)
    return true;
  if (pattern.size() > base.size())
    return false;
  for (int i = 0; i <= base.size() - pattern.size(); i++) {
    bool match = true;
    for (int j = 0; j < pattern.size(); j++) {
      if (base[i + j] != pattern[j]) {
        match = false;
        break;
      }
    }
    if (match)
      return true;
  }
  return false;
}
int string_hash(const std::string str) {
  std::uint64_t hash = 5381; // Starting value
  for (char c : str) {
    hash =
        ((hash << 5) + hash) + static_cast<unsigned char>(c); // hash * 33 + c
  }
  return static_cast<int>(hash % 2147483647); // Fit into 32-bit signed int
}
std::string replace(std::string str, std::string pattern, std::string replace) {
  if (pattern.empty())
    return str; // Avoid infinite loop if the 'from' string is empty
  size_t startPos = 0;
  while ((startPos = str.find(pattern, startPos)) != std::string::npos) {
    str.replace(startPos, pattern.length(), replace);
    startPos += replace.length(); // Move past the replacement
  }
  return str;
}
bool endswith(std::string target, std::string pattern) {
  int pointer = 0, targetSize = target.size(), patternSize = pattern.size();
  if (targetSize < patternSize)
    return false;
  while (pointer < patternSize) {
    if (target[targetSize - 1 - pointer] != pattern[patternSize - 1 - pointer])
      return false;
    pointer++;
  }
  return true;
}
std::string str_format(const char *format, ...) {
  // Initialize the variable argument list
  va_list args;
  va_start(args, format);
  // Compute the size needed for the formatted string
  va_list argsCopy;
  va_copy(argsCopy, args); // create a copy of the va_list
  size_t size = std::vsnprintf(nullptr, 0, format, argsCopy) +
                1;  // +1 for null terminator
  va_end(argsCopy); // Clean up the copied va_list
  // Allocate a buffer of the required size
  std::vector<char> buffer(size);
  // Format the string into the buffer
  std::vsnprintf(buffer.data(), size, format, args);
  // Clean up the original va_list
  va_end(args);
  // Return the result as a std::string
  return std::string(buffer.data());
}

bool listdir(std::string dirpath, std::function<void(std::string)> f) {
#ifdef _WIN32
  std::wstring wdirpath = std::filesystem::u8path(dirpath).wstring();
  if (fs::exists(wdirpath) && fs::is_directory(wdirpath)) {
    for (auto entry : std::filesystem::directory_iterator(wdirpath)) {
#else
  if (fs::exists(dirpath) && fs::is_directory(dirpath)) {
    for (auto entry : std::filesystem::directory_iterator(dirpath)) {
#endif
      f(entry.path().string());
    }
    return true;
  } else
    return false;
}

bool mkdir(std::string path) { return fs::create_directories(path); }

bool open_folder_dialog(std::string title, std::string &selectedFolder) {
  auto ret = tinyfd_selectFolderDialog(title.c_str(), "./");
  if (ret == NULL)
    return false;
  selectedFolder = ret;
  return true;
}

bool save_file_dialog(std::string title,
                      std::vector<const char *> filterPatterns,
                      std::string description, std::string &selectedFile) {
  auto ret = tinyfd_saveFileDialog(title.c_str(), "./", filterPatterns.size(),
                                   filterPatterns.data(), description.c_str());
  if (ret == NULL)
    return false;
  selectedFile = ret;
  return true;
}

bool open_file_dialog(std::string title,
                      std::vector<const char *> filterPatterns,
                      std::string description, std::string &selectedFile) {
  auto ret =
      tinyfd_openFileDialog(title.c_str(), "./", filterPatterns.size(),
                            filterPatterns.data(), description.c_str(), 0);
  if (ret == NULL)
    return false;
  selectedFile = ret;
  return true;
}

}; // namespace toolkit