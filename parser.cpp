#include <iosfwd>
#include <vector>
#include <sstream>
#include "parser.h"

namespace Parser {

std::vector<Terminal> parse_instance(std::istream& input)
{
  std::vector<Terminal> terminals;
  std::size_t n = 0;
  input >> n;
  for (std::size_t i = 0; i < n; i++) {
    input >> std::ws;
    if (input.eof()) {
      throw std::runtime_error("Malformed input: Input contains less than " + std::to_string(n) + " data rows.");
    }
    std::string line_buffer;
    std::getline(input, line_buffer);
    std::istringstream line_stream(line_buffer);

    Coordinate x = 0;
    Coordinate y = 0;
    line_stream >> x >> y >> std::ws;
    terminals.emplace_back(Terminal{ x, y });
    if (not line_stream.eof()) {
      throw std::runtime_error("Malformed input: Data line contains more than 2 numbers.");
    }
  }
  input >> std::ws;
  if (not input.eof()) {
    throw std::runtime_error("Malformed input: Input contains more than " + std::to_string(n) + " data rows.");
  }
  return terminals;
}
} // namespace Parser
