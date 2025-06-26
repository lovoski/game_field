#include <CLI11.hpp>

int main(int argc, char **argv) {
  CLI::App app{"This is a test program for c++ command line parser library."};
  std::string file;
  app.add_option("-f,--file", file, "Require an existing file")->required();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    std::cout << e.get_name() << std::endl;
    app.exit(e);
  }
  return 0;
}