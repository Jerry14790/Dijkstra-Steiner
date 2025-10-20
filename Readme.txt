See https://www.or.uni-bonn.de/lectures/ss25/cd_ex/p01.pdf for the assignment description.

Please type g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror *.cpp to compile the program.

To run the executable, type ./a.out ./example/EXAMPLE.txt

We use the bounding box model as a feasible lower bound. 

We implement a binary heap to store all pairs (v, I) not in P such that l(v, I) < infinity. 

To keep track of change of locations of elements of the heap, we use an unordered map (i.e. a hash table) to store the locations of heap elements, and update the locations whenever a DecreaseKey, Insert, or extractMin changes the tree structure of the heap.

