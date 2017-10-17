
def take_n_steps(n, testmaze, testrobot, robot_pos):

    # Mapping of robot's local coordinate system to global maze coordinate system
    dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u']}
    dir_mapping = {'u': ['l', 'u', 'r','d'], 'r': ['u', 'r', 'd','l'],
               'd': ['r', 'd', 'l','u'], 'l': ['d', 'l', 'u','r']}
    dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
    dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

    total_time = 0
    run_active = True
    while run_active:
        # check for end of time
        total_time += 1
        if total_time > n:
            run_active = False
            break

        # provide robot with sensor information, get actions
        sensing = [testmaze.dist_to_wall(robot_pos['location'], heading) 
                    for heading in dir_sensors[robot_pos['heading']]]
        rotation, movement = testrobot.next_move(sensing)

        # check for a reset
        if (rotation, movement) == ('Reset', 'Reset'):
            run_active = False
            break

        # perform rotation
        if rotation == -90:
            robot_pos['heading'] = dir_sensors[robot_pos['heading']][0]
        elif rotation == 90:
            robot_pos['heading'] = dir_sensors[robot_pos['heading']][2]
        elif rotation == 0:
            pass
        else:
            print "Invalid rotation value, no rotation performed."

        # perform movement
        if abs(movement) > 3:
            print "Movement limited to three squares in a turn."
        movement = max(min(int(movement), 3), -3) # fix to range [-3, 3]
        while movement:
            if movement > 0:
                if testmaze.is_permissible(robot_pos['location'], robot_pos['heading']):
                    robot_pos['location'][0] += dir_move[robot_pos['heading']][0]
                    robot_pos['location'][1] += dir_move[robot_pos['heading']][1]
                    movement -= 1
                else:
                    print "Movement stopped by wall."
                    movement = 0
            else:
                rev_heading = dir_reverse[robot_pos['heading']]
                if testmaze.is_permissible(robot_pos['location'], rev_heading):
                    robot_pos['location'][0] += dir_move[rev_heading][0]
                    robot_pos['location'][1] += dir_move[rev_heading][1]
                    movement += 1
                else:
                    print "Movement stopped by wall."
                    movement = 0
	return robot_pos