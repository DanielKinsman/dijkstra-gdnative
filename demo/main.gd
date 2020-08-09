extends Node2D

var Dijkstra = preload("res://bin/dijkstra.gdns")
onready var dijkstra = Dijkstra.new()

func _ready():
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
