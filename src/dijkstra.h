#ifndef GD_DIJKSTRA_H
#define GD_DIJKSTRA_H

#include <Godot.hpp>
#include <Object.hpp>
#include <Vector2.hpp>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace godot {
    class Dijkstra;
}

namespace dijkstra {
typedef pair<int, int> NodeDistance;

struct DijkstraResult {
    unordered_map<int, int> distances;
    unordered_map<int, int> previous;
    unordered_map<int, godot::Vector2> flow_field;
};

struct CompareDistance {
    bool operator()(const NodeDistance& left, const NodeDistance& right) const {
        return left.second > right.second;  // > because in the priority queue, we want top() to be the smallest
    }
};


DijkstraResult solve(godot::Dijkstra& graph);
godot::Vector2 calculate_flow(int node, const unordered_map<int, int>& neighbours, const unordered_map<int, godot::Vector2>& positions, const unordered_map<int, int>& distances);

}

namespace godot {

class Dijkstra : public Object {
    GODOT_CLASS(Dijkstra, Object)

    friend dijkstra::DijkstraResult dijkstra::solve(Dijkstra& graph);
public:
    static void _register_methods();

    Dijkstra();
    ~Dijkstra();

    void _init(); // our initializer called by Godot
    void add_node(int id, Vector2 position=Vector2());
    void remove_node(int id);
    PoolIntArray get_nodes();
    Vector2 get_position(int id);
    void add_edge(int from, int to, int weight=1);
    void remove_edge(int from, int to);
    PoolIntArray get_neighbours(int id);
    int get_weight(int from, int to);
    void solve_async(int source, Dictionary additional_node_costs);
    int get_next_node_towards_source(int id);
    Vector2 get_next_node_position_towards_source(int id);
    int get_distance_to_source(int id);
    Vector2 get_flow(int id);

private:
    unordered_set<int> node_set;
    unordered_map<int, Vector2> positions;

    // Edges are undirected. from < to always
    unordered_map<int, unordered_map<int, int>> edges; // edges[from][to] = weight

    dijkstra::DijkstraResult solve_result;

    condition_variable solving;
    mutex solve_mutex;
    mutex graph_mutex;
    thread solve_thread;
    int source = -1;
    unordered_map<int, int> additional_node_costs;
};

}

#endif
