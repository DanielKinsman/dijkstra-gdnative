#include "main.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <queue>
#include <unordered_set>
#include <unordered_map>

using namespace std;

int main() {
    dijkstra_test();
    return 0;
}

void dijkstra_test() {
    unordered_set<CellCoord> vertex_set;
    unordered_map<CellCoord, int> distances;
    unordered_map<CellCoord, CellCoord> previous;
    int rows = 50;
    int columns = 50;

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            auto coord = CellCoord(i, j);
            distances[coord] = 99999999;
            //previous[coord] = CellCoord.Invalid();
            vertex_set.insert(coord);
        }
    }

    distances[CellCoord(0, 0)] = 0;

    while(!vertex_set.empty()) {
        CellCoord current = min_distance(distances, vertex_set);
        if(vertex_set.count(current) == 0) {
            return;
        }

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
                }
            }
        }
    }
}

CellCoord min_distance(unordered_map<CellCoord, int> const& distances, unordered_set<CellCoord> const& vertex_set) {
    CellCoord min_coord;
    int min =  99999999;
    for(auto p : distances) {
        auto current = p.first;
        if(vertex_set.count(p.first) && p.second < min) {
            min_coord=p.first;
            min = p.second;
        }
    }

    assert(min != 99999999);
    return min_coord;
}
