#include "dijkstra.h"

using namespace std;
using namespace godot;

void Dijkstra::_register_methods() {
    register_method("dummy", &Dijkstra::dummy);
}

Dijkstra::Dijkstra() {
}

Dijkstra::~Dijkstra() {
}

void Dijkstra::_init() {
}

Variant Dijkstra::dummy() {
    return String("hello");
}
