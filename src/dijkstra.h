#ifndef GD_DIJKSTRA_H
#define GD_DIJKSTRA_H

#include <Godot.hpp>
#include <Object.hpp>

using namespace std;

namespace godot {

class Dijkstra : public Object {
    GODOT_CLASS(Dijkstra, Object)

public:
    static void _register_methods();

    Dijkstra();
    ~Dijkstra();

    void _init(); // our initializer called by Godot
    Variant dummy();
};

}

#endif
