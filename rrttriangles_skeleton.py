#!/usr/bin/env python3
#
#   rrttriangles.py
#
#   Use RRT to find a path around triangular obstacles.
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math          import pi, sin, cos, sqrt, ceil
from planarutils   import *


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set a maximum number of nodes...
#
# TODO:
dstep = 0.25 # change to 1.0 and 5.0 for part c
Nmax  = 1000


######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 28)
(ymin, ymax) = (0, 20)

# triangles = ((( 2, 6), ( 3, 2), ( 4, 6)),
#              (( 6, 5), ( 7, 7), ( 8, 5)),
#              (( 6, 9), ( 8, 9), ( 8, 7)),
#              ((10, 3), (11, 6), (12, 3)))

triangles = ((( 5, 2), ( 3, 10), ( 7, 10)),
             (( 9, 10), ( 13, 17), ( 23, 10)),
             (( 19, 4), ( 21, 8), ( 26, 9)))

(startx, starty) = ( 1, 5)
(goalx,  goaly)  = (13, 5)


######################################################################
#
#   Node Definition
#
class Node:
    # Initialize with coordinates.
    def __init__(self, x, y):
        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

        # Clear any parent information.
        self.parent = None

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    ######################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        for triangle in triangles:
            if PointInTriangle((self.x, self.y), triangle):
                return False
        return True
    
    def onWall(self):
        if (self.x == xmin or self.x == xmax or
            self.y == ymin or self.y == ymax):
            return True
        for triangle in triangles:
            if PointInTriangle((self.x, self.y), triangle) and (not PointInTriangleExclusive((self.x, self.y), triangle)):
                return True
        return False

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        for triangle in triangles:
            if SegmentCrossTriangle(((self.x, self.y), (other.x, other.y)),
                                    triangle):
                return False
        return True


######################################################################
#
#   Visualization
#
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.figure(figsize=(10, 7))
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_xticks(range(0,29))
        plt.gca().set_yticks(range(0,21))
        plt.gca().set_aspect('equal')

        # Show the triangles.
        for tr in triangles:
            plt.plot((tr[0][0], tr[1][0], tr[2][0], tr[0][0]),
                     (tr[0][1], tr[1][1], tr[2][1], tr[0][1]),
                     'k-', linewidth=2)
                        
        # plt.plot((2, 2.05, 2.1, 2),
        #          (2.5, 2, 2.5, 2.5),
        #         'k-', linewidth=2)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)

        self.drawLines()

        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    
    def drawNode(self, node, *args, **kwargs):
        plt.plot(node.x, node.y, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)

    def drawLines(self):
        lines = []
        vertices = []
        for t in triangles:
            for vertex in t:
                vertices.append(Node(vertex[0], vertex[1]))


        # calculating vertices for lines
        for v in vertices:
            lines.append(v)
            dist = 1
            for y in np.arange(v.y-dist,ymin-dist,-1*dist):
                temp = Node(v.x,y)
                if not temp.inFreespace():
                    if temp.onWall():
                        lines.append(temp)
                        break
                    break

            for y in np.arange(v.y+dist,ymax+dist,dist):
                temp = Node(v.x,y)
                if not temp.inFreespace():
                    if temp.onWall():
                        lines.append(temp)
                        break
                    break

        print(lines)

        # # calculates lines
        # for v in vert:
        #     # move up from vertex
        #     for row in range(v[0]-1,0,-1):
        #         if walls[row,v[1]] == 1:
        #             break
        #         else:
        #             lines.append((row,v[1]))
        #     # move down from vertex
        #     for row in range(v[0]+1,rows):
        #         if walls[row,v[1]] == 1:
        #             break
        #         else:
        #             lines.append((row,v[1]))


######################################################################
#
#   RRT Functions
#
def rrt(startnode, goalnode, visual):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        visual.drawEdge(oldnode, newnode, color='g', linewidth=1)

        visual.show()

    # Loop - keep growing the tree.
    while True:
        # Determine the target state.
        # TODO:
        if random.random() < 0.05: # make 0.0 for part a, change to 0.05, 0.5, and 1.0 to test part b
            targetnode = goalnode
        else:
            targetnode = Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))

        # Directly determine the distances to the target node.
        distances = np.array([node.distance(targetnode) for node in tree])
        index     = np.argmin(distances)
        nearnode  = tree[index]
        d         = distances[index]

        # Determine the next node.
        # TODO:
        dist = targetnode.distance(nearnode)
        xtogo = targetnode.x - nearnode.x
        ytogo = targetnode.y - nearnode.y
        if dist <= dstep:
            nextnode = targetnode
        else:
            nextnode = Node(nearnode.x + (dstep/dist)*xtogo, nearnode.y + (dstep/dist)*ytogo)

        # Check whether to attach.
        if nearnode.connectsTo(nextnode):
            addtotree(nearnode, nextnode)

            # If within dstep, also try connecting to the goal.  If
            # the connection is made, break the loop to stop growing.
            # TODO:
            if goalnode.distance(nextnode) <= dstep and nextnode.connectsTo(goalnode):
                addtotree(nextnode, goalnode)
                break

        # Check whether we should abort (tree has gotten too large).
        if (len(tree) >= Nmax):
            return (None, len(tree))

    # Build and return the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)
    return (path, len(tree))


# Post Process the Path
def PostProcess(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1


######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    # print('Running with step size ', dstep, ' and up to ', Nmax, ' nodes.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    # startnode = Node(startx, starty)
    # goalnode  = Node(goalx,  goaly)

    # # Show the start/goal nodes.
    # visual.drawNode(startnode, color='r', marker='o')
    # visual.drawNode(goalnode,  color='r', marker='o')
    visual.show("Showing basic world")


    # # Run the RRT planner.
    # print("Running RRT...")
    # (path, N) = rrt(startnode, goalnode, visual)

    # # If unable to connect, show what we have.
    # if not path:
    #     visual.show("UNABLE TO FIND A PATH")
    #     return

    # # Show the path.
    # print("PATH found after %d nodes" % N)
    # visual.drawPath(path, color='r', linewidth=2)
    # visual.show("Showing the raw path")


    # # Post Process the path.
    # PostProcess(path)

    # # Show the post-processed path.
    # visual.drawPath(path, color='b', linewidth=2)
    # visual.show("Showing the post-processed path")


if __name__== "__main__":
    main()
