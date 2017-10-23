from print_maze import print_maze
import numpy as np

def show_step_count(maze,path):
    cells = [["   " for x in range(maze.dim)] for row in range(maze.dim)]
    #symbol = (x for x in [" X "," O "," "])
    for i,x in enumerate(path):
        if i < 10: 
            cells[x[0]][x[1]] = " {} ".format(i)
        elif i < 100:
            cells[x[0]][x[1]] = " {}".format(i)
        else:
            cells[x[0]][x[1]] = "{}".format(i)
    steps = len(path)-1
    print("steps:{}").format(steps)
    print_maze(maze,cells)

def show_multiple_path(maze,paths):
    cells = [["   " for x in range(maze.dim)] for row in range(maze.dim)]
    #symbol = (x for x in [" X "," O "," "])
    for i,path in enumerate(paths):
        for x in list(set(path)):
            if cells[x[0]][x[1]] == "   ": 
                cells[x[0]][x[1]] = " {} ".format(i)
            else:
                cells[x[0]][x[1]] = cells[x[0]][x[1]][:2] + str(i)
    steps = np.sum([len(x) for x in paths])-1
    print("steps:{}").format(steps)
    print_maze(maze,cells)
    
def show_times_visited(maze,path):
    cells = [["   " for x in range(maze.dim)] for row in range(maze.dim)]
    #symbol = (x for x in [" X "," O "," "])
    times_visited = {x:0 for x in path}
    for x in path:
        times_visited[x] += 1
    for x in list(set(path)):
        cells[x[0]][x[1]] = " {} ".format(times_visited[x])
        
    print("steps:{}").format(len(path))
    print_maze(maze,cells)