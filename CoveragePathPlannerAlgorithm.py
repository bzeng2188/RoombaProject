
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

def TraverseCurrentCell(cellvertices, walls, path, Robot):
    
    # Traverse to corner of a cell
    while Robot.Position() != cellvertices[0]:
        currpos = Robot.Position()
        drow = (cellvertices[0][0] - currpos[0])/abs(cellvertices[0][0] - currpos[0])
        dcol = (cellvertices[0][1] - currpos[1])/abs(cellvertices[0][1] - currpos[1])
        Robot.Command(drow, dcol)
        path.append((currpos[0] + drow, currpos[1] + dcol))
    
    # Verify that we are in the corner of a cell
    if Robot.Position() != cellvertices[0]:
        print("ERROR, POSITION IS NOT CORRECT, I AM NOT AT THE CORNER OF A CELL")
    
    # If command = 1, go up. If command = -1, go down. If command = 0, cell complete. 
    repeat = True
    while repeat:
        currpos = Robot.Position()
        if walls[newpos[0] + 1, newpos[1]] == 1:
            command = -1
            print('Traversing Downwards')
            repeat = False
        elif walls[newpos[0] - 1, newpos[1]] == 1:
            command = 1
            print('Traversing Upwards')
            repeat = False
        elif walls[newpos[0], newpos[1] - 1] == 1:
            Robot.Command(0,1)
            path.append((currpos[0], currpos[1] + 1))
            print('Traversing Rightwards')
    # While cell is not complete... checked based on whether robot has reached all vertices of the cell
    tempvertices = []
    for vertex in cellvertices:
            tempvertices.append(vertex)

    while command != 0:
        Robot.Command(command, 0)
        newpos = Robot.Position
        path.append(newpos)
        currpos = Robot.Position()
        if currpos in tempvertices:
            tempvertices.pop(currpos)
        if len(tempvertices) == 0:
            command = 0
            break 
        if walls[newpos[0] + command, newpos[1]] == 1:
            rightmove = False
            if walls[newpos[0] + command, newpos[1]] == 1 and walls[newpos[0], newpos[1] - 1] == 1:
                Robot.Command(0,1)
                path.append((currpos[0], currpos[1] + 1))
                print('Traversing Rightwards, after running into a wall')
                rightmove = True
            if rightmove == False:
                command *= -1
            print('Reversing Traverse Direction')
        else:
            print("Continuining in Original Direction")


            

        


