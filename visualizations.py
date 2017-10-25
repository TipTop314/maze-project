import numpy as np

def print_maze(maze,cells):
    
    # Initialize maze with walls everywhere
    horizontal_lines = [["+---" for x in range(maze.dim)] 
                        for row in range(maze.dim)]
    vertical_lines = [["|" for x in range(maze.dim)] 
                        for row in range(maze.dim)]

    # Remove wall when maze tells us the wall is permissable
    for row in range(maze.dim):
        for col in range(maze.dim):
            if maze.is_permissible((row,col),'l'):
                horizontal_lines[row][col] = "    "
            if maze.is_permissible((row,col),'d'):
                vertical_lines[row][col] = " "

    # First column of 
                
    # Create generators 
    g_horizontal_lines = (x for x in horizontal_lines)
    g_vertical_lines = (x for x in vertical_lines)
    g_cells = (x for x in cells)

    n_rows = len(horizontal_lines)

    # Print top line
    newline1 = "        "
    newline2 = "         "
    for i in range(maze.dim):
        if i == maze.dim/2-1:
            newline1 += "LEFT"
        else:
            newline1 += "    " 
        if i < 10:
            newline2 += " {}  ".format(i)
        else:
            newline2 += " {} ".format(i)
    print newline1
    print newline2

    # Print walls and cell values
    for i in range(maze.dim):
        
        # Print DOWN indicator at center
        if i == maze.dim/2-1:
            newline = "DOWN    "
        else:
            newline = "        "
        # Print wall
        for wall in next(g_horizontal_lines):
            newline += wall
        # Finish wall and show UP indicator at center
        if i == maze.dim/2-1:
            print newline + "+  UP"
        else:
            print newline + "+"
        
        if i < 10:
            newline = "     {}  ".format(i)
        else:
            newline = "     {} ".format(i)
        
        for wall,cell in zip(next(g_vertical_lines),next(g_cells)):
            newline += wall + cell
        print newline + "|"

    # Print bottom wall
    newline = "        "
    for wall in horizontal_lines[0]:
        newline += wall
    print newline + "+"

    #Print RIGHT side indicator
    newline = "        "
    for i in range(maze.dim):
        if i == maze.dim/2-1:
            newline += "RIGHT"
        else:
            newline += "    " 
    print(newline)


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