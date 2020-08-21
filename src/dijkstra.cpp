#include "dijkstra.h"

#include <future>
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
    register_method("solve_async", &Dijkstra::solve_async);
    register_method("get_next_node_towards_source", &Dijkstra::get_next_node_towards_source);
    register_method("get_next_node_position_towards_source", &Dijkstra::get_next_node_position_towards_source);
    register_method("get_distance_to_source", &Dijkstra::get_distance_to_source);
    register_method("get_flow", &Dijkstra::get_flow);
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

    solve_result = dijkstra::solve(source, *this);
}

void Dijkstra::solve_async(int source) {
    if(node_set.count(source) == 0) {
        ERR_PRINT(String("{source} not in set of nodes").format(Dictionary::make("source", source)));
        return;
    }

    set_result_from_future();
    if(!solve_future.valid()) // don't start another if one is already running
        solve_future = async(launch::async, dijkstra::solve, source, ref(*this));
}

void Dijkstra::set_result_from_future() {
    if(!solve_future.valid())
        return;

    if(!have_first_result || solve_future.wait_for(chrono::seconds(0)) == future_status::ready) {
        solve_result = solve_future.get();
        have_first_result = true;
        solve_future = future<dijkstra::DijkstraResult>();  // invalidate it
    }
}

int Dijkstra::get_next_node_towards_source(int id) {
    if(node_set.count(id) == 0) {
        ERR_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return -1;
    }

    set_result_from_future();
    return solve_result.previous[id];
}

Vector2 Dijkstra::get_next_node_position_towards_source(int id) {
    return positions[get_next_node_towards_source(id)];
}

int Dijkstra::get_distance_to_source(int id) {
    if(node_set.count(id) == 0) {
        ERR_PRINT(String("{id} not in set of nodes").format(Dictionary::make("id", id)));
        return -1;
    }

    set_result_from_future();
    return solve_result.distances[id];
}

Vector2 Dijkstra::get_flow(int id) {
    set_result_from_future();
    if(solve_result.flow_field.count(id) == 0) {
        ERR_PRINT(String("no flow calculated for node {id}").format(Dictionary::make("id", id)));
        return Vector2();
    }

    return solve_result.flow_field.at(id);
}

dijkstra::DijkstraResult dijkstra::solve(int source, const Dijkstra& graph) {
    // TODO this is unsafe! Put a mutex around nodes, edges and positions
    auto node_set = unordered_set<int>(graph.node_set);
    auto edges = unordered_map<int, unordered_map<int, int>>(graph.edges);
    auto positions = unordered_map<int, Vector2>(graph.positions);
    auto result = DijkstraResult();

    result.previous[source] = source;

    unordered_set<int> working_node_set; // just for faster lookups, worth it?
    priority_queue<NodeDistance, vector<NodeDistance>, CompareDistance> node_queue;
    node_queue.push(NodeDistance(source, 0));

    for(auto node : node_set) {
        result.distances[node] = node == source ? 0 : numeric_limits<int>::max();
        working_node_set.insert(node);
    }

    while(!node_queue.empty()) {
        int current = node_queue.top().first;
        node_queue.pop();
        working_node_set.erase(current);

        for(auto node_weight_pair : edges[current]) {
            int neighbour = node_weight_pair.first;
            int weight = node_weight_pair.second;

            if(working_node_set.count(neighbour) == 0)
                continue;

            int distance_to_neighbour = result.distances[current] + weight;
            if(distance_to_neighbour < result.distances[neighbour]) {
                result.distances[neighbour] = distance_to_neighbour;
                result.previous[neighbour] = current;
                node_queue.push(NodeDistance(neighbour, distance_to_neighbour));
            }
        }
    }

    for(auto node : node_set) {
        // could do this in parallel for each node individually, but we are
        // probably on a background thread already (assuming solve_async)
        // and won't someone think of the thread startup overhead children
        result.flow_field[node] = calculate_flow(node, edges[node], positions, result.distances);
    }

    return result;
}

Vector2 dijkstra::calculate_flow(int node, const unordered_map<int, int>& neighbours, const unordered_map<int, Vector2>& positions, const unordered_map<int, int>& distances) {
    Vector2 result;

    for(auto neighbour_pair : neighbours) {
        // Why is it neighbour pos - node pos and not the other way around? :shrug:
        Vector2 offset = positions.at(neighbour_pair.first) - positions.at(node);
        auto distance_to_offset = distances.at(neighbour_pair.first);

        if(offset.y == 0.0)
            result.x -= offset.x * distance_to_offset;
        else
            result.x -= offset.x * 0.5 * distance_to_offset;

        if(offset.x == 0)
            result.y -= offset.y * distance_to_offset;
        else
            result.y -= offset.y * 0.5 * distance_to_offset;
    }

    return result;
}
