extends Node2D

var Dijkstra = preload("res://bin/dijkstra.gdns")
onready var stress_graph = Dijkstra.new()
onready var rng = RandomNumberGenerator.new()
const STRESS_NODES = 100 * 100
const STRESS_CONNECTIONS_PER_NODE = 8

func _ready():
    test()
    setup_stress()

func test():
    var dijkstra = Dijkstra.new()
    dijkstra.add_node(1)
    dijkstra.add_node(2)
    dijkstra.add_node(3)
    dijkstra.add_node(4)
    dijkstra.add_edge(1, 2, 1)
    dijkstra.add_edge(1, 3, 1)
    dijkstra.add_edge(3, 4, 1)
    dijkstra.add_edge(1, 4, 99)
    dijkstra.solve(1)
    print("nodes")
    print(dijkstra.get_nodes())
    print(dijkstra.get_nodes())
    print("paths")
    for n in dijkstra.get_nodes():
        #print("--- Node %s ---" % n) # bug in godot???
        var crap = "--- Node %s ---" % n
        print(crap)
        print("Neighbours: %s" % dijkstra.get_neighbours(n))
        var next = dijkstra.get_next_node_towards_source(n)
        print("Path %s -> %s" % [n, next])
        print("Weight %s -> %s: %s" % [n, next, dijkstra.get_weight(n, next)])
        print("Distance %s: %s" % [n, dijkstra.get_distance_to_source(n)])

func _process(_delta):
    stress_graph.solve(rng.randi_range(0, STRESS_NODES - 1))
    $Label.text = "Solved %s nodes %s edges in %ss, %s FPS" % [
        STRESS_NODES,
        STRESS_NODES * STRESS_CONNECTIONS_PER_NODE,
        Performance.get_monitor(Performance.TIME_PROCESS),
        Performance.get_monitor(Performance.TIME_FPS)
    ]

func setup_stress():
    stress_graph.add_node(0)
    for i in range(STRESS_NODES):
        for j in range(STRESS_CONNECTIONS_PER_NODE / 2):
            var neighbour = i + j + 1
            if neighbour < STRESS_NODES:
                stress_graph.add_node(neighbour)
                stress_graph.add_edge(i, neighbour, rng.randi_range(1, 100))
