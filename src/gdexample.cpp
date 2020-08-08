#include "gdexample.h"
#include <algorithm>
#include <cstdio>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <unordered_map>

using namespace std;
using namespace godot;

void GDExample::_register_methods() {
    register_method("_process", &GDExample::_process);
}

GDExample::GDExample() {
}

GDExample::~GDExample() {
    // add your cleanup here
}

void GDExample::_init() {
    // initialize any variables here
    time_passed = 0.0;
}

void GDExample::_process(float delta) {
    //if(time_passed == 0.0)
    godot::dijkstra_test();

    time_passed += delta;

    Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 10.0 + (10.0 * cos(time_passed * 1.5)));

    set_position(new_position);
}

void godot::dijkstra_test() {
    unordered_set<CellCoord> vertex_set;
    unordered_map<CellCoord, int> distances;
    unordered_map<CellCoord, CellCoord> previous;
    int rows = 100;
    int columns = 100;

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
            Godot::print(String("current (") + String::num(current.row) + String(", ") + String::num(current.column) + String(") not in vertex_set!"));
            return;
        }

        vertex_set.erase(current);
        //Godot::print(String("current (") + String::num(current.row) + String(", ") + String::num(current.column) + String(")"));

        for(int i : {-1, 0, 1}) {
            for(int j : {-1, 0, 1}) {
                if(i == 0 && j == 0)
                    continue;

                auto neighbour = CellCoord(current.row + i, current.column + j);
                if(vertex_set.count(neighbour) == 0)
                    continue;

                int distance_to_neighbour = distances[current] + 1; // TODO weight function instead of 1
                if(distance_to_neighbour < distances[neighbour]) {
                    //Godot::print(String("neighbour (") + String::num(neighbour.row) + String(", ") + String::num(neighbour.column) + String("): ") + String::num(distance_to_neighbour));
                    distances[neighbour] = distance_to_neighbour;
                    previous[neighbour] = current;
                }
            }
        }
    }
}

CellCoord godot::min_distance(unordered_map<CellCoord, int> const& distances, unordered_set<CellCoord> const& vertex_set) {
    CellCoord min_coord;
    int min =  99999999;
    for(auto p : distances) {
        auto current = p.first;
        //Godot::print(String("p (") + String::num(current.row) + String(", ") + String::num(current.column) + String(")"));
        if(vertex_set.count(p.first) && p.second < min) {
            min_coord=p.first;
            min = p.second;
        }
    }

    if(min == 99999999)
        Godot::print("oops");

    return min_coord;
}
