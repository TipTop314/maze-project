def get_known_cell_values(testrobot,testmaze):
    cells = [["   " for x in range(testmaze.dim)] for row in range(testmaze.dim)]
    for r,row in enumerate(testrobot.U_table):
        for c,val in enumerate(row):
            if (r,c) in testrobot.next_locations_table:
                if val > 0:
                    if val > 99:
                        cells[r][c] = "+99"
                    else:
                        cells[r][c] = "+" + str(val*10)[:2]
                else:
                    if abs(val) > 99:
                        cells[r][c] = "-99"
                    else:
                        cells[r][c] = str(val*10)[:3]
    return cells

def get_known_wander_values(testrobot,testmaze):
    cells = [["   " for x in range(testmaze.dim)] for row in range(testmaze.dim)]
    for r,row in enumerate(testrobot.wander_utility):
        for c,val in enumerate(row):
            if (r,c) in testrobot.wander_locations:
                if val > 0:
                    if val > 99:
                        cells[r][c] = "+" + str(val)[:1] + "h"
                    else:
                        cells[r][c] = "+" + str(val)[:2]
                else:
                    if abs(val) > 99:
                        cells[r][c] = "-99"
                    else:
                        cells[r][c] = str(val)[:3]
    return cells

def get_known_wander_percentiles(testrobot,testmaze):
    cells = [["   " for x in range(testmaze.dim)] for row in range(testmaze.dim)]
    total_utility = sum(sum(testrobot.wander_utility))
    for r,row in enumerate(testrobot.wander_utility):
        for c,val in enumerate(row):
            if (r,c) in testrobot.wander_locations:
                cells[r][c] = str(100*val/total_utility)[:3]
    return cells

def get_all_cell_values(testrobot,testmaze):
    cells = [["   " for x in range(testmaze.dim)] for row in range(testmaze.dim)]
    for r,row in enumerate(testrobot.U_table):
        for c,val in enumerate(row):
            if val > 0:
                if val > 99:
                    cells[r][c] = "+99"
                else:
                    cells[r][c] = "+" + str(val*10)[:2]
            else:
                if abs(val) > 99:
                    cells[r][c] = "-99"
                else:
                    cells[r][c] = str(val*10)[:3]
    return cells
    
def get_visited_cells(testrobot,testmaze):
    cells = [["   " for x in range(testmaze.dim)] for row in range(testmaze.dim)]
    for x in testrobot.next_locations_table:
        if testrobot.next_locations_table[x]:
            cells[x[0]][x[1]] = " X "
    return cells