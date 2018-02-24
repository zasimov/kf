#ifndef __PARSER_H
#define __PARSER_H

#include <istream>

#include "ground_truth.h"
#include "measurement_package.h"

bool parse_lazer(std::istream &, struct measurement *);
bool parse_radar(std::istream &, struct measurement *);
bool parse_ground_truth(std::istream &, struct ground_truth *);
bool parse(std::istream &, struct measurement *, struct ground_truth *);

#endif
