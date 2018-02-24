/*
 * Read measurements from file and check each line.
 */

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "parser.h"
#include "measurement_package.h"

void die(char *filename, int lineno, const std::string &msg) {
  std::cerr << "error: " << filename << ": " << lineno << ": " << msg << std::endl;
  exit(1);
};

void test_file_or_die(char *filename) {
  std::ifstream infile(filename);

  std::string line;

  struct measurement m;
  struct ground_truth g;

  int lineno = 1;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (not parse(iss, &m, &g)) {
      die(filename, lineno, "malformed line");
    }

    lineno++;
  }
}


int main(int argc, char **argv) {
  for (int i = 1; i < argc; i++) {
    test_file_or_die(argv[i]);
  }
  return 0;
}
