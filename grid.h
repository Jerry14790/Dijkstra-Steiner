#include "common.h"
#include <vector>
#include <map>
#include <bitset>
#include <unordered_map>

namespace Grid
{

struct VertexTerminalsPairAndL
{
    Coordinate x;
    Coordinate y;
    std::bitset<MAX_NUM_TERMINALS> terminals;
    int L;
    int lb; // we store the feasible lower bound (we use the bounding box here) lb(v, T-I) of a pair, so it is calculated only once.
    bool operator>(const VertexTerminalsPairAndL& other) const {
        return L + lb > other.L + other.lb;
    }
};

struct VertexTerminalsPair
{
    Coordinate x;
    Coordinate y;
    std::bitset<MAX_NUM_TERMINALS> terminals;
    bool operator==(const VertexTerminalsPair &other) const
    { return (x == other.x
              && y == other.y
              && terminals == other.terminals);
    }
    bool operator<(const VertexTerminalsPair& other) const {
        return x < other.x || (x == other.x && y < other.y) || (x == other.x && y == other.y && terminals.to_string() < other.terminals.to_string());
    }
};

struct Hasher
{
  std::size_t operator()(const VertexTerminalsPair& k) const
  {
    using std::size_t;
    using std::hash;
    using std::string;

    return ((hash<int>()(k.x)
             ^ (hash<int>()(k.y) << 1)) >> 1)
             ^ (hash<string>()(k.terminals.to_string()) << 1);
  }
};


class GridGraph
{
public:
//The main algorithm
Coordinate RunDijkstraSteiner();


GridGraph(std::vector<Terminal> terminals);


private:
//heap operations
void insert(VertexTerminalsPairAndL key);
VertexTerminalsPairAndL extractMin();
void decreaseKey(int i, int newValue);

//Use the bounding box model to compute a feasible lower bound
int FeasibleLowerBound(VertexTerminalsPair v);

std::vector<Terminal> T;
Terminal t;

// P records l-values of those pairs (v, I) which are added to P
std::map<VertexTerminalsPair, int> P;


// The all possible x or y coordinates of the Hanan grid
std::vector<Coordinate> x_values, y_values;

// return the index of an x-coordinate or y-coordinate given x_values or y_values
std::map<Coordinate, std::size_t> x_pos, y_pos;

void removeDuplicates(std::vector<Coordinate>& v);

// pos maps a vertex-terminals pair (v, I) such that ( l(v,I)<infinity and (v,I) not in P ) to its location in the heap 
std::unordered_map<VertexTerminalsPair, int, Hasher> pos;


//Heap data structure that stores pairs (v, I) such that l(v,I)<infinity and (v,I) not in P
std::vector<VertexTerminalsPairAndL> array;
void heapify(int i);

//Take a VertexTerminalsPairAndL and restrict it to VertexTerminalsPair
VertexTerminalsPair res(VertexTerminalsPairAndL var);

//Swap two variables of type VertexTerminalsPairAndL in the heap, and record their change of position in pos
void SwapAndRecordChangeOfLocation(VertexTerminalsPairAndL& var1, VertexTerminalsPairAndL& var2);

void recursion(std::size_t iterations, std::bitset<MAX_NUM_TERMINALS> bit_set, VertexTerminalsPairAndL vI);

//helper data structure to keep track of sets J in step 7
std::vector<std::size_t> places_outside_I_and_t;

};



}
