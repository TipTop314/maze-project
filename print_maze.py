
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
    
    # Create generators 
    g_horizontal_lines = (x for x in horizontal_lines)
    g_vertical_lines = (x for x in vertical_lines)
    g_cells = (x for x in cells)
    
    # Print walls and cell values
    for i in range(maze.dim):
        newline = ""
        for wall in next(g_horizontal_lines):
            newline += wall
        print newline + "+"

        newline = ""
        for wall,cell in zip(next(g_vertical_lines),next(g_cells)):
            newline += wall + cell
        print newline + "|"
    
    # Print bottom line
    newline = ""
    for wall in horizontal_lines[0]:
        newline += wall
    print newline + "+"