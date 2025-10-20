#include <fstream>
#include <iostream>
#include <span>
#include "parser.h"
#include "algorithm.h"
#include "common.h"

void print_usage(std::span<char const*> args)
{
  std::cout << "Calculates the length of a minimum Steiner Tree for the given "
               "instance file."
            << "Usage: " << args[0] << " file" << std::endl;
}

int main(int argc, char const* argv[])
{
  auto const args = std::span(argv, argc);
  if (argc != 2) {
    print_usage(args);
    return 1;
  }
  try {
    std::ifstream input_file(args[1]);
    if (input_file.bad() or input_file.fail()) {
      throw std::runtime_error("Cannot open file: " + std::string(args[1]));
    }
    auto const terminals = Parser::parse_instance(input_file);

    if (terminals.size() > MAX_NUM_TERMINALS) {
      throw std::runtime_error("Cannot handle instances with more than 20 terminals.");
    }

    std::cout << Algorithm::dijkstra_steiner(terminals) << std::endl;
  } catch (std::exception const& e) {
    std::cout << "Exception occurred: " << e.what() << std::endl;
    return 1;
  }

  return EXIT_SUCCESS;
}


