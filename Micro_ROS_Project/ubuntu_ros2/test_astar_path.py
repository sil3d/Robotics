#!/usr/bin/env python3
"""Unit tests for astar_path and path_to_waypoints in mission_engine.py"""

import math
import sys

sys.path.insert(0, '.')

# Import standalone — pas besoin de rclpy pour les fonctions A*
from mission_engine import astar_path, path_to_waypoints


class _FakeTagMap:
    """Minimal stub de TagMapSLAM pour les tests."""
    def __init__(self, tags: dict):
        self.tags = tags  # {tid: {"x": cm, "y": cm, ...}}


def _make_map(positions: dict) -> _FakeTagMap:
    """Construit une fake map depuis {tid: (x_cm, y_cm)}."""
    return _FakeTagMap({tid: {"x": x, "y": y, "views": 3, "conf": 1.0}
                        for tid, (x, y) in positions.items()})


def test_same_start_goal():
    """start == goal → retourne [start]"""
    tm = _make_map({1: (0, 0), 2: (100, 0)})
    path = astar_path(tm, 1, 1)
    assert path == [1], f"Expected [1], got {path}"
    print("[PASS] same start == goal")


def test_direct_path_two_nodes():
    """Deux nœuds → chemin direct [start, goal]"""
    tm = _make_map({1: (0, 0), 2: (100, 0)})
    path = astar_path(tm, 1, 2)
    assert path == [1, 2], f"Expected [1, 2], got {path}"
    print("[PASS] direct path two nodes")


def test_shortest_path_three_nodes():
    """
    Layout :  1 --100cm-- 2 --100cm-- 3
    Chemin 1→3 = [1, 2, 3] (coût 200) ou [1, 3] direct si graphe complet.
    Graphe complet → A* prend direct 1→3 (coût 200cm en ligne droite).
    """
    tm = _make_map({1: (0, 0), 2: (100, 0), 3: (200, 0)})
    path = astar_path(tm, 1, 3)
    assert path[0] == 1 and path[-1] == 3, f"Path must start=1 end=3, got {path}"
    print("[PASS] shortest path three nodes (collinear)")


def test_path_avoids_missing_node():
    """Si start ou goal absent de la map → retourne []"""
    tm = _make_map({1: (0, 0), 2: (100, 0)})
    assert astar_path(tm, 1, 99) == [], "Missing goal should return []"
    assert astar_path(tm, 99, 2) == [], "Missing start should return []"
    print("[PASS] missing node returns []")


def test_empty_map():
    """Map vide → retourne []"""
    tm = _FakeTagMap({})
    assert astar_path(tm, 1, 2) == [], "Empty map should return []"
    print("[PASS] empty map returns []")


def test_path_to_waypoints_conversion():
    """path_to_waypoints convertit cm → mètres correctement"""
    tm = _make_map({1: (0, 0), 2: (100, 50), 3: (200, 100)})
    tag_path = [1, 2, 3]
    wps = path_to_waypoints(tm, tag_path)
    assert len(wps) == 3
    assert wps[0] == (0.0, 0.0),  f"Expected (0.0, 0.0), got {wps[0]}"
    assert wps[1] == (1.0, 0.5),  f"Expected (1.0, 0.5), got {wps[1]}"
    assert wps[2] == (2.0, 1.0),  f"Expected (2.0, 1.0), got {wps[2]}"
    print("[PASS] path_to_waypoints cm→m conversion")


def test_path_to_waypoints_empty():
    """path_to_waypoints avec chemin vide → retourne []"""
    tm = _make_map({1: (0, 0)})
    assert path_to_waypoints(tm, []) == []
    print("[PASS] path_to_waypoints empty path")


def test_four_node_grid():
    """
    Grille 2x2 :
      1(0,0)   2(100,0)
      3(0,100) 4(100,100)
    Graphe complet → 1→4 : chemin direct ou via 2/3.
    Coût direct : 141.4cm. Via 2 : 100+100=200. Via 3 : 100+100=200.
    A* choisit le direct [1, 4].
    """
    tm = _make_map({1: (0, 0), 2: (100, 0), 3: (0, 100), 4: (100, 100)})
    path = astar_path(tm, 1, 4)
    assert path[0] == 1 and path[-1] == 4
    # Sur graphe complet, le chemin optimal est direct
    assert len(path) == 2, f"Expected direct [1,4], got {path}"
    print("[PASS] four node grid direct path")


if __name__ == '__main__':
    print("Running astar_path tests...")
    test_same_start_goal()
    test_direct_path_two_nodes()
    test_shortest_path_three_nodes()
    test_path_avoids_missing_node()
    test_empty_map()
    test_path_to_waypoints_conversion()
    test_path_to_waypoints_empty()
    test_four_node_grid()
    print("\n[SUCCESS] All astar_path tests passed!")
