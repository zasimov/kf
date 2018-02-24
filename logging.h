#ifndef LOG_H
#define LOG_H

#include <iostream>

namespace logging {

  void info(const std::string &message);
  void error(const std::string &message);
  void fatal(const std::string &message);

} // namespace logging

#endif /* LOG_H */
