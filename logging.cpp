#include <cstdlib>

#include "logging.h"

namespace logging {

  void info(const std::string &message) {
    std::cout << "info: " << message << std::endl;
  }

  void error(const std::string &message) {
    std::cerr << "error: " << message << std::endl;
  };

  void fatal(const std::string &message) {
    std::cerr << "fatal: " << message << std::endl;
    exit(1);
  }

}
