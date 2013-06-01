#!/usr/bin/env python
#-*- coding:utf-8 -*-
# main.py


import time
import sys
from gridmap import GridNode, GridMap


class Algo():
    def __init__(self, grid):
        self.visited = []
        self.calctime = 0
        self.grid = grid
        self.result = []

    def print_result(self):
        print 'Number of visited nodes: ' + str(len(self.visited))
        print 'Number of nodes in result path: ' + str(len(self.result))
        print 'Calculate time: ' + str(self.calctime)
        self.grid.print_grid()

    def save_result(self, name='result.txt'):
        f = open(name, 'w')
        f.write('Number of visited nodes: ' + str(len(self.visited)))
        f.write('\n')
        f.write('Calculate time: ' + str(self.calctime) + '\n')
        f.close()
        self.grid.save_grid(name)


class AStar(Algo):
    def __init__(self, grid):
        self.openset = []
        self.closedset = []
        self.visited = []
        self.result = []
        self.calctime = 0
        self.grid = grid

    def calc_path(self):
        pass

    def heuristic(self, curr):
        pass


class Dijkstra(Algo):
    #def __init__(self, grid):
    #    pass

    def calc_path(self):
        pass


class BestFirst(Algo):
    #def __init__(self, grid):
    #    pass

    def calc_path(self):
        pass

    def heuristic(self, curr):
        pass


def usage():
    print 'usage'


if __name__ == '__main__':
    if not sys.argv[2:]:
		usage()
		exit(0)

    grid = GridMap(40, 20)
    grid.randomize()
    print 'This grid map will be used'
    grid.print_grid()
    
    bfs = BestFirst(grid)
    dijk = Dijkstra(grid)
    astar = AStar(grid)

    bfs.calc_path()
    dijk.calc_path()
    astar.calc_path()

    print
    print 'Result of Best-First-Search'
    bfs.print_result()
    print
    print 'Result of Dijkstra algorithm'
    dijk.print_result()
    print
    print 'Result of A* algorithm'
    astar.print_result()
