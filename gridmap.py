#!/usr/bin/env python
#-*- coding:utf-8 -*-
# gridmap.py


class GridNode():
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.is_obs = False
        self.in_result = False
        self.is_start = False
        self.is_goal = False
        self.previous = None


class GridMap():
    "Square grid map"
    def __init__(self, row, col, diagonal=False):
        "make row * col gridmap. from (0, 0) to (row - 1, col - 1)"
        self.can_diagonal_move = diagonal
        self.matrix = [[GridNode(i, j) for i in xrange(row)] for j in xrange(col)]
    
    def put_single_obs(self, x, y):
        "put an obstacle on (x, y)"
        print 'Node that will be changed to an obstacle: ' + str((x, y))
        self.matrix[y][x].is_obs = True

    def put_multiple_obs(self, *args):
        print 'Nodes that will be changed to obstacles:',
        print args
        for x, y in args:
        	self.matrix[y][x].is_obs = True

    def remove_single_obs(self, x, y):
        self.matrix[y][x].is_obs = False

    def remove_multiple_obs(self, *args):
        for x, y in args:
        	self.matrix[y][x].is_obs = False

    def set_start(self, x, y):
        self.matrix[y][x].is_start = True
        self.start = self.matrix[y][x]

    def set_goal(self, x, y):
        self.matrix[y][x].is_goal = True
        self.goal = self.matrix[y][x]

    def randomize(self):
        from random import random, choice
        notobs = []
        for row in self.matrix:
        	for node in row:
        		if random() < 0.2:
        			node.is_obs = True
        		else:
        			notobs.append(node)
        s = choice(notobs) 
        s.is_start = True
        notobs.remove(s)
        g = choice(notobs)
        g.is_goal = True
        self.start = s
        self.goal = g

    def print_grid(self):
        for row in self.matrix:
        	for node in row:
        		if node.is_obs:
        		    print '\033[31m#\033[0m',
        		elif node.is_start:
        		    print '\033[36mS\033[0m',
        		elif node.is_goal:
        		    print '\033[36mG\033[0m',
        		elif node.in_result:
        		    print '\033[32m*\033[0m',
        		else:
        			print '\033[37mo\033[0m',
        	print

    def save_grid(self, name='result.txt'):
        f = open(name, 'a')
        for row in self.matrix:
        	for node in row:
        		if node.is_obs:
        		    f.write('# ')
        		elif node.is_start:
        		    f.write('S ')
        		elif node.is_goal:
        		    f.write('G ')
        		elif node.in_result:
        		    f.write('* ')
        		else:
        			f.write('o ')
        f.close()


if __name__ == '__main__':
    grid = GridMap(30, 15)
    grid.print_grid()
    print
    grid.put_multiple_obs((2, 4), (1, 1), (1, 2), (1, 3), (10, 3), (11, 3), (12, 3), (13, 3))
    grid.print_grid()
    print
    grid = GridMap(40, 20)
    grid.randomize()
    grid.print_grid()
