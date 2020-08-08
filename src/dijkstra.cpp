#include "dijkstra.h"

using namespace std;
using namespace godot;

void Dijkstra::_register_methods() {
    register_method("add_node", &Dijkstra::add_node);
    register_method("remove_node", &Dijkstra::remove_node);
    register_method("get_position", &Dijkstra::get_position);
    register_method("add_edge", &Dijkstra::add_edge);
    register_method("remove_edge", &Dijkstra::remove_edge);
    register_method("solve", &Dijkstra::solve);
    register_method("get_next_node_towards_source", &Dijkstra::get_next_node_towards_source);
    register_method("get_next_node_position_towards_source", &Dijkstra::get_next_node_position_towards_source);
}

Dijkstra::Dijkstra() {
}

Dijkstra::~Dijkstra() {
}

void Dijkstra::_init() {
}

void Dijkstra::add_node(int id, Vector2 position) {
}

void Dijkstra::remove_node(int id) {
}

Vector2 Dijkstra::get_position(int id) {
    return Vector2();
}

void Dijkstra::add_edge(int from, int to, int weight) {
}

void Dijkstra::remove_edge(int from, int to) {
}

void Dijkstra::solve(int source) {
}

int Dijkstra::get_next_node_towards_source(int id) {
    return 0;
}

Vector2 Dijkstra::get_next_node_position_towards_source(Vector2 position) {
    return Vector2();
}
