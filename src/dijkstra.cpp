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
}

void Dijkstra::add_node(int id, Vector2 position) {
    // allow if already exists; only way to update position
    node_set.insert(id);
    positions[id] = position;
}

void Dijkstra::remove_node(int id) {
    if(node_set.count(id) == 0) {
        WARN_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return;
    }

    node_set.erase(id);
    positions.erase(id);

    PoolIntArray neighbours = get_neighbours(id); // TODO maybe use an iterator
    for(int i = 0; i < neighbours.size(); i++)
        remove_edge(id, neighbours[i]);

    edges.erase(id);
}

PoolIntArray Dijkstra::get_nodes() {
    auto result = PoolIntArray();
    for(int node : node_set)
        result.append(node);

    return result;
}

Vector2 Dijkstra::get_position(int id) {
    if(node_set.count(id) == 0) {
        ERR_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return Vector2();
    }

    return positions[id];
}

void Dijkstra::add_edge(int from, int to, int weight) {
    if(weight <= 0) {
        ERR_PRINT(String("edge weight {weight} cannot be negative and should not be zero").format(Dictionary::make("weight", weight)));
        weight = 1;
    }

    if(node_set.count(from) == 0) {
        ERR_PRINT(String("{from} not in set of nodes").format(Dictionary::make("from", from)));
        return;
    }

    if(node_set.count(to) == 0) {
        ERR_PRINT(String("{to} not in set of nodes").format(Dictionary::make("to", to)));
        return;
    }

    // undirected, add both for easy lookup
    edges[from][to] = weight;
    edges[to][from] = weight;
}

void Dijkstra::remove_edge(int from, int to) {
    if(node_set.count(from) == 0) {
        ERR_PRINT(String("{from} not in set of nodes").format(Dictionary::make("from", from)));
        return;
    }

    if(node_set.count(to) == 0) {
        ERR_PRINT(String("{to} not in set of nodes").format(Dictionary::make("to", to)));
        return;
    }

    // don't care if the edges don't exist, this won't throw
    edges[from].erase(to);
    edges[to].erase(from);
}

PoolIntArray Dijkstra::get_neighbours(int id) {
    auto result = PoolIntArray();

    if(node_set.count(id) == 0) {
        ERR_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return result;
    }

    for(auto node_weight_pair : edges[id])
        result.append(node_weight_pair.first);

    return result;
}

int Dijkstra::get_weight(int from, int to) {
    if(node_set.count(from) == 0) {
        ERR_PRINT(String("{from} not in set of nodes").format(Dictionary::make("from", from)));
        return 1;
    }

    if(node_set.count(to) == 0) {
        ERR_PRINT(String("{to} not in set of nodes").format(Dictionary::make("to", to)));
        return 1;
    }

    return edges[from][to];
}

void Dijkstra::solve(int source) {
    if(node_set.count(source) == 0) {
        ERR_PRINT(String("{source} not in set of nodes").format(Dictionary::make("source", source)));
        return;
    }

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
    if(node_set.count(id) == 0) {
        ERR_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return -1;
    }

    return previous[id];
}

Vector2 Dijkstra::get_next_node_position_towards_source(Vector2 position) {
    // TODO implement
    return Vector2();
}
