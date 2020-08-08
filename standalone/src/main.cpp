#include "main.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_set>
#include <unordered_map>

using namespace std;

int main() {
    int rows = 1000;
    int columns = 10;
    auto result = dijkstra_test(rows, columns);

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            cout << result.first[CellCoord(i, j)] << "\t";
        }
        cout << endl;
    }

    return 0;
}

pair<unordered_map<CellCoord, int>, unordered_map<CellCoord, CellCoord>> dijkstra_test(int rows, int columns) {
    unordered_set<CellCoord> vertex_set; // just for faster lookups, worth it?
    priority_queue<CellDistance, vector<CellDistance>, CompareDistance> vertex_queue;
    unordered_map<CellCoord, int> distances;
    unordered_map<CellCoord, CellCoord> previous;

    auto source = CellCoord(0, 0);
    vertex_queue.push(CellDistance(source, 0));

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            auto coord = CellCoord(i, j);
            distances[coord] = coord == source ? 0 : numeric_limits<int>::max();
            vertex_set.insert(coord);
        }
    }

    while(!vertex_queue.empty()) {
        CellCoord current = vertex_queue.top().first;
        vertex_queue.pop();
        vertex_set.erase(current);

        for(int i : {-1, 0, 1}) {
            for(int j : {-1, 0, 1}) {
                if(i == 0 && j == 0)
                    continue;

                auto neighbour = CellCoord(current.row + i, current.column + j);
                if(vertex_set.count(neighbour) == 0)
                    continue;

                int distance_to_neighbour = distances[current] + 1; // TODO weight function instead of 1
                if(distance_to_neighbour < distances[neighbour]) {
                    distances[neighbour] = distance_to_neighbour;
                    previous[neighbour] = current;
                    vertex_queue.push(CellDistance(neighbour, distance_to_neighbour)); 
                }
            }
        }
    }

    return pair<unordered_map<CellCoord, int>, unordered_map<CellCoord, CellCoord>>(distances, previous);
}
