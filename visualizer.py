# grid-demo.py

from tkinter import *
import numpy as np
from PIL import ImageTk
from PIL import Image
import math
from Roomba import Roomba

def init(data):
    data.count = 0
    data.obstacle_grid = np.array([[False]*data.width for i in range(data.height)])
    data.rows = 60
    data.cols = 60
    data.img_id = None
    data.img_file = './roomba.png'
    data.pil_img = Image.open(data.img_file).resize((data.width//data.cols, data.height//data.rows))
    data.roomba_img = None

    with open(data.map_path, 'r') as f:
        for line in f:
            args = line.split(' ')
            if args[0] == 'r':
                data.obstacle_grid[int(args[2]):int(args[4])+1, int(args[1]):int(args[3])+1] = True
            elif args[0] == 'start':
                data.roomba_col = int(args[1])
                data.roomba_row = int(args[2])
                data.roomba_theta = int(args[3])
            elif args[0] == 'goal':
                data.goal_col = int(args[1])
                data.goal_row = int(args[2])
                data.goal_theta = int(args[3])

def pointInGrid(x, y, data):
    # return True if (x, y) is inside the grid defined by data.
    return x <= data.width and y <= data.height

def isCollision(x, y, data):
    # return True if (x, y) is inside the grid defined by data.
    return data.obstacle_grid[y][x]

def getCell(x, y, data):
    # aka "viewToModel"
    # return (row, col) in which (x, y) occurred or (-1, -1) if outside grid.
    if (not pointInGrid(x, y, data)):
        return (-1, -1)
    gridWidth  = data.width
    gridHeight = data.height
    cellWidth  = gridWidth / data.cols
    cellHeight = gridHeight / data.rows
    row = y // cellHeight
    col = x // cellWidth
    # triple-check that we are in bounds
    row = min(data.rows-1, max(0, row))
    col = min(data.cols-1, max(0, col))
    return (int(row), int(col))

def getCellBounds(row, col, data):
    # aka "modelToView"
    # returns (x0, y0, x1, y1) corners/bounding box of given cell in grid
    gridWidth  = data.width
    gridHeight = data.height
    columnWidth = gridWidth / data.cols
    rowHeight = gridHeight / data.rows
    x0 = col * columnWidth
    x1 = (col+1) * columnWidth
    y0 = row * rowHeight
    y1 = (row+1) * rowHeight
    return (x0, y0, x1, y1)

def mousePressed(event, canvas, data):
    (row, col) = getCell(event.x, event.y, data)
    # should we check whether the row col is on top of the roomba?
    if not data.obstacle_grid[row][col]:
        (x0, y0, x1, y1) = getCellBounds(row, col, data)
        data.obstacle_grid[row][col] = True
        canvas.create_rectangle(x0, y0, x1, y1, fill="cyan")
        # Update roomba map

def keyPressed(event, data):
    pass

def timerFired(data):
    pass

def drawInitialMap(canvas, data):
    # draw grid of cells
    for row in range(data.rows):
        for col in range(data.cols):
            (x0, y0, x1, y1) = getCellBounds(row, col, data)
            fill = "cyan" if (data.obstacle_grid[row][col]) else "orange"
            canvas.create_rectangle(x0, y0, x1, y1, fill=fill)
    # draw roomba
    data.tkimage = ImageTk.PhotoImage(data.pil_img.rotate(data.roomba_theta))
    (x0, y0, x1, y1) = getCellBounds(data.roomba_row, data.roomba_col, data)
    data.img_id = canvas.create_image(x0+(x1-x0)//2, y0+(y1-y0)//2, image=data.tkimage)


def redrawAll(canvas, data):
    step = data.path[data.count]
    data.roomba_theta = (data.roomba_theta + math.degrees(step[2])) % 360
    data.roomba_col += step[0]
    data.roomba_row += step[1]
    data.tkimage = ImageTk.PhotoImage(data.pil_img.rotate(data.roomba_theta))
    (x0, y0, x1, y1) = getCellBounds(data.roomba_row, data.roomba_col, data)
    data.img_id = canvas.create_image(x0+(x1-x0)//2, y0+(y1-y0)//2, image=data.tkimage)

    if data.count < len(data.path):
        data.count += 1

####################################
# use the run function as-is
####################################

def run(map_path='./map1.txt', width=300, height=300):
    def redrawAllWrapper(canvas, data):
        if data.count < len(data.path):
            if data.img_id != None:
                canvas.delete(data.img_id)
            redrawAll(canvas, data)
            canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, canvas, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Visualizer(object): pass
    data = Visualizer()
    data.width = width
    data.height = height
    data.timerDelay = 300 # milliseconds
    data.map_path = map_path
    init(data)

    roomba = Roomba()
    roomba.setStart((data.roomba_col, data.roomba_row, math.radians(data.roomba_theta)))
    roomba.setGoal((data.goal_col, data.goal_row, math.radians(data.goal_theta)))
    roomba.setMap(data.obstacle_grid)
    print((data.roomba_col, data.roomba_row, math.radians(data.roomba_theta)))
    print((data.goal_col, data.goal_row, math.radians(data.goal_theta)))

    # create the root and the canvas
    root = Tk()
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.pack()
    drawInitialMap(canvas, data)

    # generate initial plan
    data.path = roomba.findPath()
    print(data.path)

    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")

run('map1.txt', 720, 720)