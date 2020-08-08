#include "dijkstra.h"

#include <queue>

using namespace std;
using namespace godot;

void Dijkstra::_register_methods() {
    register_method("add_node", &Dijkstra::add_node);
    register_method("remove_node", &Dijkstra::remove_node);
    register_method("get_nodes", &Dijkstra::get_nodes);
    register_method("get_position", &Dijkstra::get_position);
    register_method("add_edge", &Dijkstra::add_edge);
    register_method("remove_edge", &Dijkstra::remove_edge);
    register_method("get_neighbours", &Dijkstra::get_neighbours);
    register_method("get_weight", &Dijkstra::get_weight);
    register_method("solve", &Dijkstra::solve);
    register_method("get_next_node_towards_source", &Dijkstra::get_next_node_towards_source);
    register_method("get_next_node_position_towards_source", &Dijkstra::get_next_node_position_towards_source);
}

Dijkstra::Dijkstra() {
}

Dijkstra::~Dijkstra() {
}

void Dijkstra::_init() {
    node_set.clear();
    positions.clear();
}

void Dijkstra::add_node(int id, Vector2 position) {
    // TODO check if already in node_set
    node_set.insert(id);
    positions[id] = position;
}

void Dijkstra::remove_node(int id) {
    // TODO check if not in node set
    node_set.erase(id);
    positions.erase(id);

    PoolIntArray neighbours = get_neighbours(id); // TODO maybe use an iterator
    for(int i = 0; i < neighbours.size(); i++)
        remove_edge(id, neighbours[i]);
}

PoolIntArray Dijkstra::get_nodes() {
    auto result = PoolIntArray();
    for(int node : node_set)
        result.append(node);

    return result;
}

Vector2 Dijkstra::get_position(int id) {
    return positions[id];
}

void Dijkstra::add_edge(int from, int to, int weight) {
    // undirected, add both for easy lookup
    // TODO make sure from and to are in node_set
    edges[from][to] = weight;
    edges[to][from] = weight;
}

void Dijkstra::remove_edge(int from, int to) {
    // TODO make sure from and to are in node_set
    // TODO what to do if edges don't exist?
    edges[from].erase(to);
    edges[to].erase(from);
}

PoolIntArray Dijkstra::get_neighbours(int id) {
    auto result = PoolIntArray();
    for(auto node_weight_pair : edges[id])
        result.append(node_weight_pair.first);
    
    return result;
}

int Dijkstra::get_weight(int from, int to) {
    return edges[from][to];
}

void Dijkstra::solve(int source) {
    distances.clear();
    previous.clear();
    previous[source] = source;

    unordered_set<int> working_node_set; // just for faster lookups, worth it?
    priority_queue<NodeDistance, vector<NodeDistance>, CompareDistance> node_queue;
    node_queue.push(NodeDistance(source, 0));

    for(auto node : this->node_set) {
        distances[node] = node == source ? 0 : numeric_limits<int>::max();
        working_node_set.insert(node);
    }

    while(!node_queue.empty()) {
        int current = node_queue.top().first;
        node_queue.pop();
        working_node_set.erase(current);

        PoolIntArray neighbours = get_neighbours(current); // TODO maybe use an iterator
        for(int i = 0; i < neighbours.size(); i++) {
            int neighbour = neighbours[i];
            if(working_node_set.count(neighbour) == 0)
                continue;

            int distance_to_neighbour = distances[current] + get_weight(current, neighbour);
            if(distance_to_neighbour < distances[neighbour]) {
                distances[neighbour] = distance_to_neighbour;
                previous[neighbour] = current;
                node_queue.push(NodeDistance(neighbour, distance_to_neighbour));
            }
        }
    }
}

int Dijkstra::get_next_node_towards_source(int id) {
    // TODO handle non-existence
    return previous[id];
}

Vector2 Dijkstra::get_next_node_position_towards_source(Vector2 position) {
    // TODO implement
    return Vector2();
}
