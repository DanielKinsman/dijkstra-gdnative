#ifndef GD_DIJKSTRA_H
#define GD_DIJKSTRA_H

#include <Godot.hpp>
#include <Object.hpp>
#include <Vector2.hpp>

using namespace std;

namespace godot {

class Dijkstra : public Object {
    GODOT_CLASS(Dijkstra, Object)

public:
    static void _register_methods();

    Dijkstra();
    ~Dijkstra();

    void _init(); // our initializer called by Godot
    void add_node(int id, Vector2 position=Vector2());
    void remove_node(int id);
    Vector2 get_position(int id);
    void add_edge(int from, int to, int weight=1);
    void remove_edge(int from, int to);
    void solve(int source);
    int get_next_node_towards_source(int id);
    Vector2 get_next_node_position_towards_source(Vector2 position);
};

}

#endif
