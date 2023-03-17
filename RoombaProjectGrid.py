#!/usr/bin/env python3
#
#   hw5_localize.py
#
#   Homework 5 code framework to localize a robot in a grid...
#
#   Places to edit are marked as TODO.
#
import numpy as np
import time
from HelperFunctions import Visualization, Robot
from CoveragePathPlannerAlgorithm import *


#
#  Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                  XxxxxxxxxxX                  x',
     'x                   xxxxxxxxx                   x',
     'x      XxxxxxxX      xxxxxxx                    x',
     'x     xxxxxxxxxx      xxxxx                     x',
     'x    xxxxxxxxxxxx      xxx                      x',
     'x   XxxxxxxxxxxxxX      X                       x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                                               x',
     'x                             XxxxxxxxX         x',
     'x                            xxxxxxxxx          x',
     'x                           xxxxxxxxx           x',
     'x                          xxxxxxxxx            x',
     'x                         XxxxxxxxX             x',
     'x                                               x',
     'x                                               x',
     'x            X                                  x',
     'x           xxx                                 x',
     'x          XxxxX                                x',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

walls = np.array([[1.0*(c == 'x' or c == "X") for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)
vertices = np.array([[1.0*(c == "X") for c in s] for s in w])

#
#  Prediction
#
#    bel         Grid of probabilities (current belief)
#    drow, dcol  Delta in row/col
#    probCmd     Modeled probability of command executing
#    prd         Grid of probabilities (prediction)
#
def computePrediction(bel, drow, dcol, probCmd):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols))

    # Iterate over/determine the probability for all (non-wall) elements.
    for row in range(1, rows-1):
        for col in range(1, cols-1):
            if (drow != 0 or dcol != 0) and walls[row - drow, col - dcol] == 0:
                prd[row, col] = bel[row - drow, col - dcol] * probCmd + bel[row, col] * (1-probCmd)
                if walls[row,col] == 1:
                    prd[row,col] = 0
                    prd[row - drow, col - dcol] = bel[row - drow, col - dcol] * probCmd
            # TODO.... if ... prd[row, col] ...

    # Return the prediction grid
    return prd


#
#  Measurement Update (Correction)
#
#    prior       Grid of prior probabilities (belief)
#    probSensor  Grid of probability that (sensor==True)
#    sensor      Value of sensor
#    post        Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    # Update the belief based on the sensor reading, which can be one
    # of two cases: (sensor==True) or (sensor==False)
    post = np.zeros((rows,cols))
    if (sensor):
        post = probSensor * prior
        # post = TODO
    else:
        post = (1 - probSensor) * prior
        # post = TODO

    # Normalize.
    s = np.sum(post)
    if (s == 0.0):
        print("LOST ALL BELIEF.  SHOULD NOT HAPPEN.  STARTING OVER!!!!")
        post = 1.0 - walls
        s    = np.sum(post)

    post = (1.0/s) * post
    return post


#
#  Pre-compute the Sensor Probability Grid
#
#    drow, dcol    Direction in row/col
#    probProximal  List of probability that sensor triggers at dist=(index+1)
#    prob          Grid of probability that (sensor==True)
#
def precomputeSensorProbability(drow, dcol, probProximal):
    # Prepare an empty probability grid.
    prob = np.zeros((rows, cols))

    # Pre-compute the probability for each grid element, knowing the
    # walls and the direction of the sensor.
    for row in range(1, rows-1):
        for col in range(1, cols-1):
            # if (drow != 0 or dcol != 0) and walls[row + drow, col + dcol] == 1:
            #     prob[row, col] = probProximal[0]
            for dist in range(1, len(probProximal)+1):
                if (drow != 0 or dcol != 0) and walls[row + dist*drow, col + dist*dcol] == 1:
                    prob[row, col] = probProximal[dist-1]
                    break
            # TODO ....  prob[row, col] = ....

    # Return the computed grid.
    return prob

def trapdecomp():
    lines = [] #stores the coordinates of the lines that separate the cells
    vert = [] # stores the coordinates of each obstacle's vertices
    cells = [] # stores the cells, where each cell is an array of the coordinates of the cell's vertices
    
    # calculates vert from global variable vertics
    for i in range(len(vertices)):
        for j in range(len(vertices[i])):
            if vertices[i][j] == 1:
                vert.append((i,j))

    # calculates lines
    for v in vert:
        # move up from vertex
        for row in range(v[0]-1,0,-1):
            if walls[row,v[1]] == 1:
                break
            else:
                lines.append((row,v[1]))
        # move down from vertex
        for row in range(v[0]+1,rows):
            if walls[row,v[1]] == 1:
                break
            else:
                lines.append((row,v[1]))

    # calculates cells
    # cells = []
    # grid = walls
    # for l in lines:
    #     grid[l[0]][l[1]] = 2
    # # inside grid, 0 = free space, 1 = wall, 2 = line, 3 = other cell
    # incompleteCell = True
    # incompleteGrid = True
    # i = 1
    # j = 1

    # # while incompleteGrid:
    # #     i = 1
    # #     j = 1
    # while incompleteCell:
    #     cell = []
    #     # select top left of cell
    #     if grid[i][j] == 0:
    #         cell.append((i,j))

    #         # go down until find a cell that is free
    #         for row in range(i, rows):
    #             if grid[row][j] == 1:
    #                 cell.append((row-1,j)) # select bottom left of cell
    #                 break
        
    #     # defined a line
    #     # sweep line to the right until anything along the line sees a turqoise line or runs into a wall
    #     # look at top endpoint
    #         # if top endpoint in wall, move down until free
    #         # elif top endpoint can move up, move up until blocked
    #         # else leave it alone
    #     # look at bottom endpoint
    #         # if bottom endpoint in wall, move up until free
    #         # elif bottom endpoint can move down, move down until blocked
    #         # else leave it alone

    #     c = j
    #     for col in range(j,cols):
    #         for row in range(cell[0][0], cell[1][0]+1):
    #             if grid[row][j+1] == 1 or grid[row][j+1] == 3: # special case: vertical line
    #                 incompleteCell = False # break out of second while loop

    #             if grid[row][col] == 1 or grid[row][col] == 3: # ran into wall or other cell
    #                 print("hi")
    #                 c = col-1
    #                 break
    #             elif grid[row][col] == 2: # ran into line
    #                 print("helo")
    #                 c = col
    #                 break

    #     topright = (cell[0][0], c)
    #     bottomright = (cell[1][0], c)

    #     if grid[topright[0]][c] == 1: # topright in an obstacle
    #         tempr = topright[0]
    #         while grid[tempr][c] == 1 or grid[tempr][c] == 3:
    #             tempr -= 1 # move topright down until free
    #         topright = (tempr,c)
    #     elif grid[topright[0] + 1][c] == 0 or grid[topright[0]][c] == 2: # topright can move up
    #         tempr = topright[0]
    #         while grid[tempr][c] == 0 or grid[tempr][c] == 2:
    #             tempr += 1 # move topright up until blocked
    #         tempr -= 1
    #         topright = (tempr,c)

    #     if grid[bottomright[0]][c] == 1: # bottomright in an obstacle
    #         tempr = bottomright[0]
    #         while grid[tempr][c] == 1 or grid[tempr][c] == 3:
    #             tempr += 1 # move bottomright up until free
    #         bottomright = (tempr,c)
    #     elif grid[bottomright[0]][c] == 0 or grid[bottomright[0]][c] == 2: # topright can move up
    #         tempr = bottomright[0]
    #         while grid[tempr][c] == 0 or grid[tempr][c] == 2:
    #             tempr -= 1 # move bottomright down until blocked
    #         tempr += 1
    #         bottomright = (tempr,c)

    #     cell.append(topright)
    #     cell.append(bottomright)

    #     # make vertical lines of cell have values of 3 in grid
    #     for row in range(cell[0][0], cell[1][0]+1):
    #         grid[row][cell[0][1]] = 3
    #     for row in range(cell[2][0], cell[3][0]+1):
    #         grid[row][cell[2][1]] = 3

    #     cells.append(cell)
    #     print(cells)
    #     incompleteCell = False
        

        # OLD STUFF... SEE IF YOU STILL WANT
        # curr = cell[1] # bottom left corner
        # r = curr[0]
        # c = curr[1]
        # if grid[curr[0] - 1][curr[1] + 1] != 1: # should move in a downward diagonal line
        #     rdir = -1
        # elif grid[curr[0]][curr[1] + 1] != 1: # should move directly to the right
        #     if grid[cell[0][0]][cell[0][1] + 1] != 1:
        #         rdir = 0
        #     else: # special case were just a vertical line
        #         break
        # elif grid[curr[0] + 1][curr[1] + 1] != 1: # should move in an upward diagonal line
        #     rdir = 1
        # else:
        #     break

        # while grid[r][c] == 0: # keep going until see line or wall
        #     r += rdir
        #     c += 1
        # if grid[r][c] == 1: # if see wall, move diagonally up one cell and make that the bottom right corner
        #     cell.append((r-rdir,c-1))
        # elif grid[r][c] == 2: # if see line, make that the bottom right corner
        #     cell.append((r,c))

        # curr = cell[2] # bottom right corner
        # # go up until find a cell that is free
        # for row in range(curr[0],0,-1):
        #     if grid[row][curr[1]] == 1:
        #         cell.append((row+1,curr[1])) # select top right of cell
        #         break

        # # select next top left corner
        # curr = cell[len(cell)-1] # look at bottomright corner
        # for col in range(curr[1],cols):
        #     for row in range(0, rows):
        #         if grid[row][col] != 1 and grid[row][col] != 3:
        #             i = row
        #             j = col
        #         else:
        #             # NEED TO FIX THIS, HAVE TO MOVE BACK TO BEGINNING AND DOWN???
        #             incompleteGrid = False

    return lines, cells


def TraverseCurrentCell(cellvertices, walls, startposition, Robot):
    
    # Traverse to corner of a cell
    returnpath = Dijkstra(startposition, cellvertices[0], walls)
    
    # Verify that we are in the corner of a cell
    if returnpath[-1] != cellvertices[0]:
        print("ERROR, POSITION IS NOT CORRECT, I AM NOT AT THE CORNER OF A CELL")
    
    # If command = 1, go up. If command = -1, go down. If command = 0, cell complete. 
    #print("Starting at the location", returnpath[-1])
    tempvertices = []
    for vertex in cellvertices:
            tempvertices.append(vertex)
    if returnpath[-1] in tempvertices:
        tempvertices.pop(tempvertices.index(returnpath[-1]))
    repeat = True
    while repeat:
        #print("Getting stuck!!!!")
        newpos = returnpath[-1]
        if walls[newpos[0] + 1, newpos[1]] == 1:
            command = -1
            #print('Traversing Downwards')
            repeat = False
        elif walls[newpos[0] - 1, newpos[1]] == 1:
            command = 1
            #print('Traversing Upwards')
            repeat = False
        elif walls[newpos[0], newpos[1] - 1] == 1:
            addedpos = (newpos[0], newpos[1] + 1)
            returnpath.append(addedpos)
            #print('Traversing Rightwards')
    # While cell is not complete... checked based on whether robot has reached all vertices of the cell

    attemptright = False
    while command != 0:
        currpos = returnpath[-1]
        addedpos = (currpos[0] + command, currpos[1])
        returnpath.append(addedpos)
        currpos = returnpath[-1]
        #print("Checkpoint vertices I need to traverse through include:", tempvertices)
        #print("Current virtual position is:", currpos)
        if attemptright == True:
            if walls[currpos[0], currpos[1] + 1] == 0:
                addedpos = (currpos[0], currpos[1] + 1)
                returnpath.append(addedpos)
                currpos = returnpath[-1]
                #print('Traversing Rightwards, after running into a wall')
                attemptright = False
        if currpos in tempvertices:
            tempvertices.pop(tempvertices.index(currpos))
        if len(tempvertices) == 0:
            command = 0
        if walls[currpos[0] + command, currpos[1]] == 1:
            attemptright = True
            if walls[currpos[0], currpos[1] + 1] == 0:
                addedpos = (currpos[0], currpos[1] + 1)
                returnpath.append(addedpos)
                currpos = returnpath[-1]
                #print('Traversing Rightwards, after running into a wall')
                attemptright = False
                if currpos in tempvertices:
                    tempvertices.pop(tempvertices.index(currpos))
                if len(tempvertices) == 0:
                    command = 0
                if walls[currpos[0] + command, currpos[1]] == 0:
                    addedpos = (currpos[0] + command, currpos[1])
                    returnpath.append(addedpos)
                    currpos = returnpath[-1]
                    #print('Traversing Rightwards, after running into a wall')
                    attemptright = False
                if currpos in tempvertices:
                    tempvertices.pop(tempvertices.index(currpos))
                if len(tempvertices) == 0:
                    command = 0
            if walls[currpos[0] + command, currpos[1]] == 1:
                command *= -1
                #print('Reversing Traverse Direction')
            else:
                attemptright = False
                #print("Capable of continuining traverse direction, reaching end first before turning around")
        else:
            print("Continuining in Original Direction")
    
    return returnpath


# 
#
#  Main Code
#
def main():
    startpos = (12, 26)
    startpos  = (2, 6)
    path = [startpos]
    lines, cells = trapdecomp()

    # Initialize the figure.
    visual = Visualization(walls)

    # TODO... PICK WHAT THE "REALITY" SHOULD SIMULATE:
    # Initialize the robot simulation.
    # part (a)
    # robot  = Robot(walls)
    # Part (b)
    robot = Robot(walls, row=startpos[0], col=startpos[1])
    # Part (c)
    # robot = Robot(walls, row=12, col=26, probProximal = [0.9, 0.6, 0.3])
    # Part (d), (e)
    # robot = Robot(walls, row=15, col=47,
    #               probCmd = 0.8, probProximal = [0.9, 0.6, 0.3])
    # And to play:
    # robot = Robot(walls, probCmd = 0.8, probProximal = [0.9, 0.6, 0.3])


    # TODO... PICK WHAT YOUR LOCALIZATION SHOULD ASSUME:
    # Pick the algorithm assumptions:
    probCmd      = 1.0                  # Part (a/b), (c), (d)
    # probCmd      = 0.8                  # Part (e), or to play
    probProximal = [1.0]                # Part (a/b)
    # probProximal = [0.9, 0.6, 0.3]      # Part (c), (d), (e), or to play

    # Report.
    print("Localization is assuming probCmd = " + str(probCmd) +
          " and probProximal = " + str(probProximal))


    # Pre-compute the probability grids for each sensor reading.
    probUp    = precomputeSensorProbability(-1,  0, probProximal)
    probRight = precomputeSensorProbability( 0,  1, probProximal)
    probDown  = precomputeSensorProbability( 1,  0, probProximal)
    probLeft  = precomputeSensorProbability( 0, -1, probProximal)

    # Show the sensor probability maps.
    # visual.Show(probUp)
    # input("Probability of proximal sensor up reporting True")
    # visual.Show(probRight)
    # input("Probability of proximal sensor right reporting True")
    # visual.Show(probDown)
    # input("Probability of proximal sensor down reporting True")
    # visual.Show(probLeft)
    # input("Probability of proximal sensor left reporting True")


    # Start with a uniform belief grid.
    bel = 1.0 - walls
    bel = (1.0/np.sum(bel)) * bel

    time.sleep(15)
    # Loop continually.
    repeatedsteps = 0
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(bel, path, vertices, lines, robot.Position())

        timedelay = 0

        traversepath = TraverseCurrentCell([(8,4),(1,4),(5,7),(1,7)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(5,8),(1,8),(5,13),(1,13)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(5,14),(1,14),(1,17),(8,17)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(1,18),(23,18)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(3,19),(1,19),(1,29),(3,29)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(13,30),(1,30),(1,38),(13,38)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,39),(1,39),(1,47),(23,47)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,34),(19,34),(15,38),(23,38)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,26),(19,26),(19,33),(23,33)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,24),(10,24),(9,25),(23,25)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(17,26),(8,26),(5,29),(14,29)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,19),(5,19),(9,23),(23,23)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(20,13),(10,13),(23,17),(10,17)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,4),(10,4),(10,12),(21,12)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        traversepath = TraverseCurrentCell([(23,1),(1,1),(1,3),(23,3)], walls, robot.Position(), robot)
        for node in traversepath:
            drow = node[0] - robot.Position()[0]
            dcol = node[1] - robot.Position()[1]
            print(node)
            if node in path:
                repeatedsteps += 1
            path.append((robot.Position()[0] + drow, robot.Position()[1] + dcol))
            robot.Command(drow, dcol)
            visual.Show(bel, path, vertices, lines, robot.Position())
            time.sleep(timedelay)
        time.sleep(10)
        # Get the command key to determine the direction.
        # while True:
        #     key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
        #     if   (key == 'q'):  return
        #     elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
        #     elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
        #     elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
        #     elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break

        currrow = robot.Position()[0]
        currcol = robot.Position()[1]
        if walls[currrow + drow][currcol + dcol] != 1:
            path.append((currrow+drow, currcol+dcol))

        # Move the robot in the simulation.
        robot.Command(drow, dcol)

        # Compute a prediction.
        prd = computePrediction(bel, drow, dcol, probCmd)
        #visual.Show(prd)
        #input("Showing the prediction")

        # Check the prediction.
        if abs(np.sum(prd) - 1.0) > 1e-12:
            print("WARNING: Prediction does not add up to 100%")


        # Correct the prediction/execute the measurement update.
        bel = prd
        bel = updateBelief(bel, probUp,    robot.Sensor(-1,  0))
        bel = updateBelief(bel, probRight, robot.Sensor( 0,  1))
        bel = updateBelief(bel, probDown,  robot.Sensor( 1,  0))
        bel = updateBelief(bel, probLeft,  robot.Sensor( 0, -1))

        print("repeated steps:", repeatedsteps)

if __name__== "__main__":
    main()