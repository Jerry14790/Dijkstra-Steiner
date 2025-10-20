#include "grid.h"
#include <vector>
//#include <map>
#include <iostream>
#include <algorithm>

namespace Grid
{

int GridGraph::FeasibleLowerBound(VertexTerminalsPair v){
    int x_max = v.x, x_min = v.x, y_max = v.y, y_min = v.y;

    for(std::size_t i = 0; i < T.size(); i++){
        if (v.terminals.test(i) == 0) {
            if(T[i].x > x_max){
                x_max = T[i].x;
            }
            if(T[i].x < x_min){
                x_min = T[i].x;
            }
            if(T[i].y > y_max){
                y_max = T[i].y;
            }
            if(T[i].y < y_min){
                y_min = T[i].y;
            }
        }
    }

    return x_max - x_min + y_max - y_min;
}


Coordinate GridGraph::RunDijkstraSteiner(){
    if (T.size() == 1){
        return 0;
    }
    //step 1 of the algorithm
    std::bitset<MAX_NUM_TERMINALS> T_without_t(0); // A variable that will represent T-t
    for (std::size_t i = 1; i < T.size(); i++){
        std::bitset<MAX_NUM_TERMINALS> temp(0);
        temp.set(i);
        T_without_t.set(i);
        insert(VertexTerminalsPairAndL{T[i].x, T[i].y, temp, 0, FeasibleLowerBound(VertexTerminalsPair{T[i].x, T[i].y, temp})});
    }

    VertexTerminalsPair target = {t.x, t.y, T_without_t};
    
    //We skip step 2 of the algorithm because pairs (v, emptyset) always have l-value = infinity, so they can never be chosen as (v, I) in step 3.

    while (true) {
        //step 3
        VertexTerminalsPairAndL vI = extractMin();

        //step 4
        P.insert({res(vI), vI.L});
        
        //step 5
        if (res(vI) == target){
            return vI.L;
        }

        //step 6

        std::size_t xIndex = x_pos.at(vI.x);
        std::size_t yIndex = y_pos.at(vI.y);

        //the west neighbor of v
        if (xIndex > 0){
            VertexTerminalsPair wI = {x_values[xIndex-1], y_values[yIndex], vI.terminals};
            // we only consider (w,I) if it is not in P, because l(w,I) does not change after (w,I) is added to P
            if (P.count(wI) == 0){
                if(pos.count(wI) == 0){
                    // now we must have l(w,I)=infinity
                    insert(VertexTerminalsPairAndL{wI.x, wI.y, wI.terminals, vI.L + x_values[xIndex]- x_values[xIndex - 1], FeasibleLowerBound(wI)});
                } else if (array.at(pos.at(wI)).L > vI.L + x_values[xIndex] - x_values[xIndex - 1]){ // remove .at() of array to improve speed
                    decreaseKey(pos.at(wI), vI.L + x_values[xIndex]- x_values[xIndex - 1]);
                }
            }
        }
        // the east neighbor of v
        if (xIndex < x_values.size() - 1){
            VertexTerminalsPair wI = {x_values[xIndex + 1], y_values[yIndex], vI.terminals};
            if (P.count(wI) == 0){
                if(pos.count(wI) == 0){
                    insert(VertexTerminalsPairAndL{wI.x, wI.y, wI.terminals, vI.L + x_values[xIndex + 1] - x_values[xIndex], FeasibleLowerBound(wI)});
                } else if (array.at(pos.at(wI)).L > vI.L + x_values[xIndex + 1]- x_values[xIndex]){
                    decreaseKey(pos.at(wI), vI.L + x_values[xIndex + 1]- x_values[xIndex]);
                }
            }
        }
        // the north neighbor of v
        if (yIndex < y_values.size() - 1){
            VertexTerminalsPair wI = {x_values[xIndex], y_values[yIndex + 1], vI.terminals};
            if (P.count(wI) == 0){
                if(pos.count(wI) == 0){
                    insert(VertexTerminalsPairAndL{wI.x, wI.y, wI.terminals, vI.L + y_values[yIndex + 1] - y_values[yIndex], FeasibleLowerBound(wI)});
                } else if (array.at(pos.at(wI)).L > vI.L + y_values[yIndex + 1] - y_values[yIndex]){
                    decreaseKey(pos.at(wI), vI.L + y_values[yIndex + 1] - y_values[yIndex]);
                }
            }
        }
        // the south neighbor of v
        if (yIndex > 0){
            VertexTerminalsPair wI = {x_values[xIndex], y_values[yIndex - 1], vI.terminals};
            if (P.count(wI) == 0){
                if(pos.count(wI) == 0){
                    insert(VertexTerminalsPairAndL{wI.x, wI.y, wI.terminals, vI.L + y_values[yIndex] - y_values[yIndex - 1], FeasibleLowerBound(wI)});
                } else if (array.at(pos.at(wI)).L > vI.L + y_values[yIndex] - y_values[yIndex - 1]){
                    decreaseKey(pos.at(wI), vI.L + y_values[yIndex] - y_values[yIndex - 1]);
                }
            }
        }

        //step 7
        for (std::size_t i = 1; i < T.size(); i++){
            if(!vI.terminals.test(i)){
                places_outside_I_and_t.push_back(i);
            }
        }
        recursion(places_outside_I_and_t.size(), std::bitset<MAX_NUM_TERMINALS>(0), vI); // check it
        places_outside_I_and_t.clear();
    }
    return 0;
}


void GridGraph::recursion(std::size_t iterations, std::bitset<MAX_NUM_TERMINALS> bit_set, VertexTerminalsPairAndL vI){
    if (iterations == 0) {
        // now we have a bitset representing an arbitrary subset of T-(I union {t})
        VertexTerminalsPair vJ = {vI.x, vI.y, bit_set};
        if (bit_set != std::bitset<MAX_NUM_TERMINALS>(0) && P.count(vJ) == 1 ){ // exclude bit_set==0?
            VertexTerminalsPair v_I_union_J = {vI.x, vI.y, vI.terminals | bit_set};
            // l(v,I union J) cannot change after (v, I union J) is added to P
            if (P.count(v_I_union_J) == 0){
                if (pos.count(v_I_union_J) == 0){
                    insert(VertexTerminalsPairAndL{v_I_union_J.x, v_I_union_J.y, v_I_union_J.terminals, vI.L + P.at(vJ), FeasibleLowerBound(v_I_union_J)});
                } else if (array.at(pos.at(v_I_union_J)).L > vI.L + P.at(vJ)){
                    decreaseKey(pos.at(v_I_union_J), vI.L + P.at(vJ));
                }
            }
        }
    } else {
    recursion(iterations - 1, bit_set, vI);
    bit_set.set(places_outside_I_and_t.at(iterations - 1));
    recursion(iterations - 1, bit_set, vI);
    }
}

GridGraph::GridGraph(std::vector<Terminal> terminals)
{
    T = terminals;
    t = terminals[0];

    for(std::size_t i = 0; i < terminals.size(); i++)
    {
        x_values.push_back(terminals[i].x);
        y_values.push_back(terminals[i].y);
    }
    std::sort(x_values.begin(), x_values.end());
    std::sort(y_values.begin(), y_values.end());
    removeDuplicates(x_values);
    removeDuplicates(y_values);

    for(std::size_t i = 0; i < x_values.size(); i++)
    {
        x_pos.insert({x_values[i], i});
    }

    for(std::size_t i = 0; i < y_values.size(); i++)
    {
        y_pos.insert({y_values[i], i});
    }
}

bool operator==(const Terminal& lhs, const Terminal& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

// Function to extract the minimum element from the heap
VertexTerminalsPairAndL GridGraph::extractMin()
{
    if (array.size() <= 0) {
        throw std::underflow_error("Heap underflow");
    }

    if (array.size() == 1) {
        VertexTerminalsPairAndL root = array[0];
        pos.erase(res(array.back()));
        array.pop_back();
        return root;
    }

    SwapAndRecordChangeOfLocation(array[0], array.back());
    VertexTerminalsPairAndL root = array.back();
    pos.erase(res(array.back()));
    array.pop_back();

    heapify(0);

    return root;
}

// Helper function to maintain the heap property
void GridGraph::heapify(int i)
{
    int smallest = i;
    int left = 2 * i + 1 ;
    int right = 2 * i + 2;
    int size = array.size();

    if (left < size && array[smallest] > array[left])
        smallest = left;

    if (right < size && array[smallest] > array[right])
        smallest = right;

    if (smallest != i) {
        SwapAndRecordChangeOfLocation(array[i], array[smallest]);
        heapify(smallest);
    }
}


void GridGraph::decreaseKey(int i, int newValue)
{
    if (i >= (int)array.size() || array[i].L <= newValue) {
        throw std::invalid_argument(
            "Invalid index or new value is not smaller");
    }

    array[i].L = newValue;
    while (i != 0 && array[(i - 1) / 2] > array[i]) {
        SwapAndRecordChangeOfLocation(array[i], array[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

void GridGraph::insert(VertexTerminalsPairAndL key)
{
    array.push_back(key);
    pos.insert({res(key), array.size()-1}); // date type incompatible issue?
    int i = array.size() - 1;

    while (i != 0 && array[(i - 1) / 2] > array[i]) {
        SwapAndRecordChangeOfLocation(array[i], array[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}



void GridGraph::removeDuplicates(std::vector<Coordinate>& v) 
{ 
    std::vector<Coordinate> temp; 
  
    // If current element is not equal  
    // to next element then store that  
    // current element 
    for (int i = 0; i < (int)v.size() - 1; i++) 
        if (v[i] != v[i + 1]) 
            temp.push_back(v[i]); 
  
    // Store the last element as whether  
    // it is unique or repeated, it hasn't  
    // stored previously 
    temp.push_back(v[v.size() - 1]); 
    v = temp;
}

VertexTerminalsPair GridGraph::res(VertexTerminalsPairAndL var){
    return VertexTerminalsPair{var.x, var.y, var.terminals};
}

void GridGraph::SwapAndRecordChangeOfLocation(VertexTerminalsPairAndL& var1, VertexTerminalsPairAndL& var2){

    int tempPosition = pos[res(var2)];
    pos[res(var2)] = pos[res(var1)];
    pos[res(var1)] = tempPosition;

    VertexTerminalsPairAndL temp = var2;
    var2 = var1;
    var1 = temp;
}

} 

