extends Node2D

var Dijkstra = preload("res://bin/dijkstra.gdns")

func _ready():
    print_debug(Dijkstra.new().dummy())
