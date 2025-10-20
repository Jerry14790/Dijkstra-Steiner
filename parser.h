#ifndef PARSER_H
#define PARSER_H

#include <vector>
#include <iostream>
#include "common.h"

namespace Parser {

std::vector<Terminal> parse_instance(std::istream& input);

}

#endif /*PARSER_H */
