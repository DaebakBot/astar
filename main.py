#!/usr/bin/env python
#-*- coding:utf-8 -*-
# main.py


import time
import sys
import heapq
from gridmap import GridNode, GridMap
from random import random


class Algo():
    def __init__(self, grid):
        self.visited = []
        self.calctime = -1
        self.grid = grid
        self.result = []

    def print_result(self, printmap):
        print 'Number of visited nodes: ' + str(len(self.visited))
        print 'Number of nodes in result path: ' + str(len(self.result))
        print 'Calculate time: ' + str(self.calctime) + ' sec'
        if printmap:
            self.grid.print_grid()

    def save_result(self, name='result.txt'):
        f = open(name, 'w')
        filename.write('Number of visited nodes: ' + str(len(self.visited)))
        filename.write('\n')
        filename.write('Calculate time: ' + str(self.calctime) + '\n')
        f.close()
        self.grid.save_grid(name)

    def get_neighbors(self, vertex):
        if self.grid.can_diagonal_move:
            candidates = [(vertex.x - 1, vertex.y - 1),
                          (vertex.x - 1, vertex.y),
                          (vertex.x - 1, vertex.y + 1),
                          (vertex.x, vertex.y - 1),
                          (vertex.x, vertex.y + 1),
                          (vertex.x + 1, vertex.y - 1),
                          (vertex.x + 1, vertex.y),
                          (vertex.x + 1, vertex.y + 1)]
        else:
            candidates = [(vertex.x - 1, vertex.y),
                          (vertex.x, vertex.y + 1),
                          (vertex.x, vertex.y - 1),
                          (vertex.x + 1, vertex.y)]

        candidates = [(x, y) for (x, y) in candidates \
                            if x >= 0 and y >= 0 \
                                and x < self.grid.col \
                                and y < self.grid.row]
        result = []
        for i, j in candidates:
            result.append(self.grid.matrix[j][i])
        return result


class AStar(Algo):
    def calc_path(self, debug, filename):
        if not self.grid.start_set:
            print 'please set start and goal, or randomize'
            return
        if debug:
            filename.write('\nin A*\n')
        t = time.time()
        start = self.grid.start
        start.mindistance = 0.0
        start.h_value = self.heuristic(start)
        openset = [start]
        closedset = set()

        while openset:
            u = heapq.heappop(openset)
            self.visited.append(u)
            u.visited = True
            closedset.add(u)

            if debug:
                filename.write('u: ' + str(u.x) + ', ' + str(u.y))
                filename.write(' ' + str(time.time()) + '\n')

            if u == self.grid.goal:
                break

            for target in self.get_neighbors(u):
                if debug:
                    filename.write('target: ' + str(target.x))
                    filename.write(', ' + str(target.y))
                    filename.write(' ' + str(time.time()) + '\n')
                weight = 1.0
                g = weight + u.mindistance
                h = self.heuristic(target)
                f = g + h
                if not target.is_obs:
                    if target.isopen or target in closedset:
                        if target.mindistance > g:
                            target.h_value = f
                            target.mindistance = g
                            target.previous = u
                    else:
                        target.isopen = True
                        target.h_value = f
                        target.mindistance = g
                        target.previous = u
                        heapq.heappush(openset, target)

        self.calctime = time.time() - t
        u = self.grid.goal
        while u:
            u.in_result = True
            self.result.append(u)
            u = u.previous

    def heuristic(self, curr):
        return (abs(curr.x - self.grid.goal.x) \
                + abs(curr.y - self.grid.goal.y))


class Dijkstra(Algo):
    def calc_path(self, debug, filename):
        if not self.grid.start_set:
            print 'please set start and goal, or randomize'
            return
        if debug:
            filename.write('\nin Dijkstra\n')
        t = time.time()
        start = self.grid.start
        start.mindistance = 0.0
        start.h_value = 0.0
        priority_queue = [start]

        while priority_queue:
            u = heapq.heappop(priority_queue)
            self.visited.append(u)
            u.visited = True
            if debug:
                filename.write('u: ' + str(u.x) + ', ' + str(u.y))
                filename.write(' ' + str(time.time()) + '\n')

            if u == self.grid.goal:
                break

            for target in self.get_neighbors(u):
                if debug:
                    filename.write('target: ' + str(target.x))
                    filename.write(', ' + str(target.y))
                    filename.write(' ' + str(time.time()) + '\n')
                weight = 1.0
                g = weight + u.mindistance
                if not target.is_obs and g < target.mindistance:
                    if target in priority_queue:
                        priority_queue.remove(target)
                    target.mindistance = g
                    target.h_value = g
                    target.previous = u
                    heapq.heappush(priority_queue, target)

        self.calctime = time.time() - t
        u = self.grid.goal
        while u:
            u.in_result = True
            self.result.append(u)
            u = u.previous


class BestFirst(Algo):
    def calc_path(self, debug, filename):
        if not self.grid.start_set:
            print 'please set start and goal, or randomize'
            return
        t = time.time()
        start = self.grid.start
        start.mindistance = 0.0
        start.h_value = self.heuristic(start)
        priority_queue = [start]
        closed = set()

        while priority_queue:
            u = heapq.heappop(priority_queue)
            self.visited.append(u)
            closed.add(u)

            if u == self.grid.goal:
                break

            for target in self.get_neighbors(u):
                h = self.heuristic(target)
                if not target.is_obs and target not in closed:
                    if target in priority_queue:
                        priority_queue.remove(target)
                    target.mindistance = h
                    target.h_value = h
                    target.previous = u
                    heapq.heappush(priority_queue, target)

        self.calctime = time.time() - t
        u = self.grid.goal
        while u:
            u.in_result = True
            self.result.append(u)
            u = u.previous

    def heuristic(self, curr):
        return abs(curr.x - self.grid.goal.x) + abs(curr.y - self.grid.goal.y)


def usage():
    print 'usage'


def make_random_pick(row, col):
    result = []
    for i in xrange(row):
        for j in xrange(col):
            if random() < 0.2:
                result.append((i, j))
    return result


def main():
    if len(sys.argv) < 2:
        usage()
        exit(0)

    printmap = False
    savefile = False
    debugmode = False
    f = None

    if '-p' in sys.argv:
        printmap = True
    if '-s' in sys.argv:
        savefile = True
    if '--debug' in sys.argv or '-d' in sys.argv:
        debugmode = True
        f = open('debug.log', 'w')

    ran_pick = make_random_pick(int(sys.argv[1]), int(sys.argv[2]))

    bfs = BestFirst(GridMap(int(sys.argv[1]), int(sys.argv[2])))
    dijk = Dijkstra(GridMap(int(sys.argv[1]), int(sys.argv[2])))
    astar = AStar(GridMap(int(sys.argv[1]), int(sys.argv[2])))

    while len(ran_pick) < 2:
        ran_pick = make_random_pick(int(sys.argv[1]), int(sys.argv[2]))

    bfs.grid.put_multiple_obs(ran_pick[1:-2])
    dijk.grid.put_multiple_obs(ran_pick[1:-2])
    astar.grid.put_multiple_obs(ran_pick[1:-2])

    bfs.grid.set_start(ran_pick[0][0], ran_pick[0][1])
    dijk.grid.set_start(ran_pick[0][0], ran_pick[0][1])
    astar.grid.set_start(ran_pick[0][0], ran_pick[0][1])

    bfs.grid.set_goal(ran_pick[-1][0], ran_pick[-1][1])
    dijk.grid.set_goal(ran_pick[-1][0], ran_pick[-1][1])
    astar.grid.set_goal(ran_pick[-1][0], ran_pick[-1][1])

    print 'calc b'
    bfs.calc_path(debugmode, f)
    print 'calc d'
    dijk.calc_path(debugmode, f)
    print 'calc a'
    astar.calc_path(debugmode, f)

    print
    print 'Result of Best-First-Search'
    bfs.print_result(printmap)
    print
    print 'Result of Dijkstra algorithm'
    dijk.print_result(printmap)
    print
    print 'Result of A* algorithm'
    astar.print_result(printmap)

    if debugmode:
        f.close()

if __name__ == '__main__':
    main()
