
#!/usr/bin/env python3
#
#
#   Algorithm to find way through each cell
#
#
import numpy as np
from HelperFunctions import Visualization, Robot
import bisect
import matplotlib.pyplot as plt

class State:
    WALL      = -1      # Not a legal state - just to indicate the wall
    UNKNOWN   =  0      # "Air"
    ONDECK    =  1      # "Leaf"
    PROCESSED =  2      # "Trunk"
    PATH      =  3      # Processed and later marked as on path to goal

    def __init__(self, row, col):
        # Save the location.
        self.row = row
        self.col = col

        # Clear the status and costs.
        self.status = State.UNKNOWN

        # Clear the references.
        self.parent    = None
        self.neighbors = []
    
    def Position(self):
         return (self.row, self.col)
    

def Dijkstra(start, goal, walls):
    print("Traversing between", start, " and", goal)
    M = 25
    N = 49
    states = [[State(m,n) for n in range(N)] for m in range(M)]

    for row in range(M):
        for col in range(N):
            if walls[row][col] == 1:
                states[row][col].status = State.WALL
                
    for m in range(M):
        for n in range(N):
            if not states[m][n].status == State.WALL:
                for (m1, n1) in [(m-1,n), (m+1,n), (m,n-1), (m,n+1)]:
                    if not states[m1][n1].status == State.WALL:
                        states[m][n].neighbors.append(states[m1][n1])
    
    start = states[start[0]][start[1]]
    goal  = states[goal[0]][goal[1]]

    print("Starting Dijkstra's")

    onDeck = []
    onDeck.append(start)
    while True:
        current = onDeck.pop(0)
        # Process each neighbor of the current state
        for neighbor in current.neighbors:
            # Only add a new neighbor if it is not a wall, it is not already processed, and it is not already ondeck.
            if neighbor.status != State.WALL and neighbor.status != State.PROCESSED and neighbor.status != State.ONDECK:
                neighbor.parent = current
                # If unknown, add the unknown neighbor as as new entry in onDeck
                if neighbor.status == State.UNKNOWN:
                    neighbor.status = State.ONDECK
                    # For Dijkstra's, we use .append for FIFO. For A*, we use bisect.insort to sort list as we go. In this case, we are doing A*
                    # onDeck.append(neighbor)
                    onDeck.append(neighbor)

        if current == goal:
            current.status = State.PROCESSED
            break
        current.status = State.PROCESSED
    
    print("Dijkstra's path calculated. Attempting to traverse")

    currentstate = goal
    temppath = []
    returnpath = []
    while currentstate != start:
        currentstate.status = State.PATH
        temppath.append((currentstate.row, currentstate.col))
        currentstate = currentstate.parent
    start.status = State.PATH

    while len(temppath) > 0:
        returnpath.append(temppath.pop(-1))
    
    return returnpath
