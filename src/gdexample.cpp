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
    if(time_passed == 0.0)
        godot::dijkstra_test();

    time_passed += delta;

    Vector2 new_position = Vector2(10.0 + (10.0 * sin(time_passed * 2.0)), 10.0 + (10.0 * cos(time_passed * 1.5)));

    set_position(new_position);
}

void godot::dijkstra_test() {
    unordered_set<CellCoord> vertex_set;
    unordered_map<CellCoord, int> distances;
    unordered_map<CellCoord, CellCoord> previous;
    int rows = 2;
    int columns = 2;

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            auto coord = CellCoord(i, j);
            distances[coord] = 99999999;
            //previous[coord] = CellCoord.Invalid();
            vertex_set.insert(coord);
        }
    }

    auto min_distance_coord = CellCoord(0, 0);
    distances[min_distance_coord] = 0;
    
    while(!vertex_set.empty()) {
        CellCoord current = min_distance_coord;
        if(vertex_set.count(current) == 0) {
            Godot::print(String("current (") + String::num(current.row) + String(", ") + String::num(current.column) + String(") not in vertex_set!"));
            return;
        }

        vertex_set.erase(current);
        Godot::print(String("current (") + String::num(current.row) + String(", ") + String::num(current.column) + String(")"));

        for(int i : {-1, 0, 1}) {
            for(int j : {-1, 0, 1}) {
                if(i == 0 && j == 0)
                    continue;

                auto neighbour = CellCoord(current.row + i, current.column + j);
                if(vertex_set.count(neighbour) == 0)
                    continue;

                int distance_to_neighbour = distances[current] + 1; // TODO weight function instead of 1
                Godot::print(String("neighbour (") + String::num(neighbour.row) + String(", ") + String::num(neighbour.column) + String("): ") + String::num(distance_to_neighbour));
                Godot::print(String("min_distance_coord (") + String::num(min_distance_coord.row) + String(", ") + String::num(min_distance_coord.column) + String("): ") + String::num(distances[min_distance_coord]));
                if(distance_to_neighbour < distances[neighbour]) {
                    distances[neighbour] = distance_to_neighbour;
                    previous[neighbour] = current;
                    if(min_distance_coord == current || distance_to_neighbour < distances[min_distance_coord]) {
                        min_distance_coord = neighbour;
                        Godot::print(String("min (") + String::num(neighbour.row) + String(", ") + String::num(neighbour.column) + String("): ") + String::num(distance_to_neighbour));
                    }
                }
            }
        }
    }
}

//int godot::min_key_from_map(std::unordered_map<int, int> map) {
//    return min_element(map.begin(), map.end(), CompareSecond())->first;
//}
