#include "toolkit/utils.hpp"
#include <cctype>
#include <cstdarg>
#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
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

void copy_file(std::filesystem::path src_filepath,
               std::filesystem::path dst_filepath) {
  try {
    fs::copy_file(src_filepath, dst_filepath,
                  fs::copy_options::overwrite_existing);
    spdlog::info("copy file from {0} to {1}", src_filepath.string(),
                 dst_filepath.string());
  } catch (const fs::filesystem_error &e) {
    spdlog::error("{0}, failed to copy file from {1} to {2}", e.what(),
                  src_filepath.string(), dst_filepath.string());
  }
}
void copy_dir(std::filesystem::path src_dirpath,
              std::filesystem::path dst_dirpath) {
  try {
    fs::copy(src_dirpath, dst_dirpath,
             fs::copy_options::recursive |
                 fs::copy_options::overwrite_existing);
    spdlog::info("copy file from {0} to {1}", src_dirpath.string(),
                 dst_dirpath.string());
  } catch (const fs::filesystem_error &e) {
    spdlog::error("{0}, failed to copy file from {1} to {2}", e.what(),
                  src_dirpath.string(), dst_dirpath.string());
  }
}

bool zip_file(std::string in_filename, std::string out_filename, int level) {
  constexpr size_t CHUNK = 16384;
  std::vector<unsigned char> in(CHUNK), out(CHUNK);

  std::ifstream infile(in_filename, std::ios::binary);
  if (!infile.is_open()) {
    spdlog::error("cannot open input file {0}", in_filename);
    return false;
  }

  std::ofstream outfile(out_filename, std::ios::binary);
  if (!outfile.is_open()) {
    spdlog::error("cannot open output file {0}", out_filename);
    return false;
  }

  z_stream strm{};
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;

  if (deflateInit(&strm, level) != Z_OK) {
    spdlog::error("deflateInit failed");
    return false;
  }

  int flush;
  do {
    infile.read(reinterpret_cast<char *>(in.data()), CHUNK);
    strm.avail_in = infile.gcount();
    if (infile.bad()) {
      spdlog::error("Error reading file");
      deflateEnd(&strm);
      return false;
    }
    flush = infile.eof() ? Z_FINISH : Z_NO_FLUSH;
    strm.next_in = in.data();

    do {
      strm.avail_out = CHUNK;
      strm.next_out = out.data();
      deflate(&strm, flush);
      size_t have = CHUNK - strm.avail_out;
      outfile.write(reinterpret_cast<char *>(out.data()), have);
      if (!outfile.good()) {
        spdlog::error("Error writing compressed file");
        deflateEnd(&strm);
        return false;
      }
    } while (strm.avail_out == 0);
  } while (flush != Z_FINISH);

  deflateEnd(&strm);
  return true;
}

bool unzip_file(std::string in_filename, std::string out_filename,
                size_t buffer_size) {
  std::vector<unsigned char> in(buffer_size), out(buffer_size);

  std::ifstream infile(in_filename, std::ios::binary);
  if (!infile.is_open()) {
    spdlog::error("cannot open input file {0}", in_filename);
    return false;
  }

  std::ofstream outfile(out_filename, std::ios::binary);
  if (!outfile.is_open()) {
    spdlog::error("cannot open output file {0}", out_filename);
    return false;
  }

  z_stream strm{};
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;

  if (inflateInit(&strm) != Z_OK) {
    spdlog::error("inflateInit failed");
    return false;
  }

  int ret;
  do {
    infile.read(reinterpret_cast<char *>(in.data()), buffer_size);
    strm.avail_in = infile.gcount();
    if (infile.bad()) {
      spdlog::error("Error reading compressed file");
      inflateEnd(&strm);
      return false;
    }
    if (strm.avail_in == 0)
      break;
    strm.next_in = in.data();

    do {
      strm.avail_out = buffer_size;
      strm.next_out = out.data();
      ret = inflate(&strm, Z_NO_FLUSH);
      if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
        spdlog::error("inflate failed");
        inflateEnd(&strm);
        return false;
      }
      size_t have = buffer_size - strm.avail_out;
      outfile.write(reinterpret_cast<char *>(out.data()), have);
      if (!outfile.good()) {
        spdlog::error("Error writing decompressed file");
        inflateEnd(&strm);
        return false;
      }
    } while (strm.avail_out == 0);

  } while (ret != Z_STREAM_END);

  inflateEnd(&strm);
  return ret == Z_STREAM_END;
}

}; // namespace toolkit