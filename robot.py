import numpy as np
import random

# Mapping of robot's local coordinate system to global maze coordinate system
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
           'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u']}
dir_mapping = {'u': ['l', 'u', 'r','d'], 'r': ['u', 'r', 'd','l'],
           'd': ['r', 'd', 'l','u'], 'l': ['d', 'l', 'u','r']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
           'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

class Robot(object):
    
    def __init__(self, maze_dim, start_location, start_heading):
        
        # Robot constants
        self.maze_dim = maze_dim

        # Learned information
        self.next_locations_table = dict()
        self.dead_ends = []
        self.found_goal = False
        self.back_to_start = False

        # Present info
        self.move_count = 0
        self.current_location = start_location
        self.heading = start_heading
        self.path_taken = [start_location]
        self.path_taken_to_goal = [start_location]
        self.path_taken_to_start = []

        # Initialize tables
        self.U_table = np.zeros((self.maze_dim,self.maze_dim))
        self.maze_rewards = np.zeros((self.maze_dim,self.maze_dim))
        self.times_visited = np.zeros((self.maze_dim,self.maze_dim))
        
        self.U_table_to_start = np.zeros((self.maze_dim,self.maze_dim))
        self.goals = []
        
        self.initialize_tables()
    
    def initialize_tables(self):
        # Initialize utility table with zero in the center
        # and with increasingly negative values as you move
        # radially out from the center
        for x in [i for i in range(-self.maze_dim/2,self.maze_dim/2+1) if i != 0]:
            for y in [j for j in range(-self.maze_dim/2,self.maze_dim/2+1) if j != 0]:
                if x < 0: 
                    dx = self.maze_dim/2 
                else: 
                    dx = self.maze_dim/2 - 1
                if y < 0:
                    dy = self.maze_dim/2
                else:
                    dy = self.maze_dim/2 - 1
                self.U_table[x+dx][y+dy] = (-(abs(x)+abs(y))*2 + 4)*0.1
        
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                #create rewards
                self.maze_rewards[x][y] = -1
                #create next_locations_table
                self.next_locations_table[(x,y)] = []
                #create utility table for heading back to start of maze
                self.U_table_to_start[(x,y)] = (-x-y)*0.1
        
        #save goal locations for later
        for x in [-1, 0]:
            for y in [-1,0]:
                self.goals.append((self.maze_dim/2 + x,self.maze_dim/2 + y))
        
    def get_visible_next_locations(self,sensors):
        valid_moves = []
        valid_moves = list(sensors)

        current_location = np.array(self.current_location)

        # Convert valid moves to valid locations robot can move to 
        visible_next_locations = []
        for i,val in enumerate(valid_moves):
            if val > 0:
                for x in range(1,val+1):
                    if x <= 3:
                        if dir_mapping[self.heading][i] == 'u':
                            visible_next_locations.append(tuple((current_location + [0,x])))
                        elif dir_mapping[self.heading][i] == 'd':
                            visible_next_locations.append(tuple(current_location + [0,-x]))
                        elif dir_mapping[self.heading][i] == 'l':
                            visible_next_locations.append(tuple(current_location + [-x,0]))
                        else:
                            visible_next_locations.append(tuple(current_location + [x,0]))
                    else:
                        break
                    
        return visible_next_locations

    def get_movements(self, next_location):
        current_location = np.array(self.current_location)
        next_location = np.array(next_location)

        # Get action to get to next location and it's direction
        action = next_location - current_location
        action_dir = list(action/np.linalg.norm(action))
        action_dir = dir_move.keys()[dir_move.values().index(action_dir)]

        rotation_mapping = [-90,0,90,0]
        rotation = rotation_mapping[dir_mapping[self.heading].index(action_dir)]

        if rotation==0 and self.heading != action_dir:
            movement = -abs(sum(action))
        else:
            movement = abs(sum(action))
            self.heading = action_dir

        return rotation, movement
      
    def get_backward_locations(self):
        
        # If robot's last move was to step forward, add stepping backward up
        # to a maximum of the same number of steps to the valid moves list
        if self.move_count > 0:
            n_steps = 0
            current_location = np.array(self.current_location)
            last_location = np.array(self.path_taken[-2])

            # Get the steps and direction of the last move taken
            last_move = current_location - last_location
            if not sum(last_move)==0:
                last_move_dir = last_move / np.linalg.norm(last_move)
                # Check if last move was a forward step
                if np.array_equal(last_move_dir,dir_move[self.heading]):
                    n_steps = abs(sum(last_move))
                    dir_bwd_steps = np.array(dir_move[dir_reverse[self.heading]])
            
            # Convert number of backward steps to list of locations
            backward_locations = []
            for x in range(1,n_steps+1):
                backward_locations.append(tuple((current_location + x*dir_bwd_steps)))
            return backward_locations
        else:
            return []
        

    def update_next_locations_table(self,visible_next_locations):
        
        # Remove visible next locations which are dead ends from list
        for dead_end in self.dead_ends:
            for next_location in visible_next_locations:
                if next_location == dead_end:
                    visible_next_locations.remove(next_location)
                
        # If there doesn't appear to be any valid moves,
        # consider current location a dead end
        if not visible_next_locations and self.current_location != (0,0):
            self.remove_dead_end()
        else:
            # Create list of locations to add
            locations_to_add = list(visible_next_locations)

            # Add backward locations to add
            backward_locations = self.get_backward_locations()
            locations_to_add += backward_locations
            
            # Remove visible next locations which are already in table
            copy_locations_to_add = list(locations_to_add)
            for i in locations_to_add:
                for j in self.next_locations_table[self.current_location]:
                    if i[0] == j[0] and i[1] == j[1]:
                            copy_locations_to_add.remove(i)
            locations_to_add = list(copy_locations_to_add)
            
            # Add locations to next_locations_table        
            self.next_locations_table[self.current_location] += locations_to_add
           
    def remove_dead_end(self):
        # Add location to dead end list
        self.dead_ends.append(self.current_location)
        
        # Remove location from next_location_table
        for location in self.next_locations_table:
            for next_location in self.next_locations_table[location]:
               if next_location == self.current_location:
                    self.next_locations_table[location].remove(next_location)

    def choose_next_location(self):
        
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if not self.current_location in self.dead_ends:
            next_locations = list(self.next_locations_table[self.current_location])
        else:
            next_locations = self.get_backward_locations()
            if not next_locations:
                bwd_step = np.array(dir_move[dir_reverse[self.heading]])
                current_location = np.array(self.current_location)
                next_locations = [tuple(current_location + bwd_step)]
        
        # Find largest utility value
        maxU = -100000
        for location in next_locations:
            if self.U_table[location] > maxU:
                maxU = self.U_table[location]

        # Make list of all next locations with maxU and choose one
        maxU_locations = []
        for location in next_locations:
            if self.U_table[location] == maxU:
                maxU_locations.append(location)
        next_location = random.choice(maxU_locations)
        
        # Add maze reward to utility table to track I've been here
        self.U_table[self.current_location] += self.maze_rewards[self.current_location]
        
        return next_location

    def choose_next_location_2(self):
        
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if not self.current_location in self.dead_ends:
            next_locations = list(self.next_locations_table[self.current_location])
        else:
            next_locations = self.get_backward_locations()
            if not next_locations:
                bwd_step = np.array(dir_move[dir_reverse[self.heading]])
                current_location = np.array(self.current_location)
                next_locations = [tuple(current_location + bwd_step)]
        
        min_visited = 100
        for location in next_locations:
            if self.times_visited[location] < min_visited:
                min_visited = self.times_visited[location]
        
        min_visited_locations = []
        for location in next_locations:
            if self.times_visited[location] == min_visited:
                min_visited_locations.append(location)
                
        # Find largest utility value
        maxU = -100000
        for location in min_visited_locations:
            if self.U_table[location] > maxU:
                maxU = self.U_table[location]

        # Make list of all next locations with maxU and choose one
        maxU_locations = []
        for location in min_visited_locations:
            if self.U_table[location] == maxU:
                maxU_locations.append(location)
        next_location = random.choice(maxU_locations)
        
        return next_location  
        
    def choose_next_location_3(self):
        
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if not self.current_location in self.dead_ends:
            next_locations = list(self.next_locations_table[self.current_location])
        else:
            next_locations = self.get_backward_locations()
            if not next_locations:
                bwd_step = np.array(dir_move[dir_reverse[self.heading]])
                current_location = np.array(self.current_location)
                next_locations = [tuple(current_location + bwd_step)]
        
        min_visited = 100
        for location in next_locations:
            if self.times_visited[location] < min_visited:
                min_visited = self.times_visited[location]
        
        min_visited_locations = []
        for location in next_locations:
            if self.times_visited[location] == min_visited:
                min_visited_locations.append(location)
                
        next_location = random.choice(min_visited_locations)
        
        return next_location  
        
    def choose_next_location_4(self):
        
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if not self.current_location in self.dead_ends:
            next_locations = list(self.next_locations_table[self.current_location])
        else:
            next_locations = self.get_backward_locations()
            if not next_locations:
                bwd_step = np.array(dir_move[dir_reverse[self.heading]])
                current_location = np.array(self.current_location)
                next_locations = [tuple(current_location + bwd_step)]
                
        next_location = random.choice(next_locations)
        
        return next_location  
     
    def head_to_start(self):
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if not self.current_location in self.dead_ends:
            next_locations = list(self.next_locations_table[self.current_location])
        else:
            next_locations = self.get_backward_locations()
            if not next_locations:
                bwd_step = np.array(dir_move[dir_reverse[self.heading]])
                current_location = np.array(self.current_location)
                next_locations = [tuple(current_location + bwd_step)]
        
        #remove goal locations for choices
        for location in next_locations:
            for goal_loc in self.goals:
                if location == goal_loc:
                    next_locations.remove(location)
        
        min_visited = 100
        for location in next_locations:
            if self.times_visited[location] < min_visited:
                min_visited = self.times_visited[location]
        
        min_visited_locations = []
        for location in next_locations:
            if self.times_visited[location] == min_visited:
                min_visited_locations.append(location)
                
        # Find largest utility value
        maxU = -100000
        for location in min_visited_locations:
            if self.U_table_to_start[location] > maxU:
                maxU = self.U_table_to_start[location]

        # Make list of all next locations with maxU and choose one
        maxU_locations = []
        for location in min_visited_locations:
            if self.U_table_to_start[location] == maxU:
                maxU_locations.append(location)
        next_location = random.choice(maxU_locations)
        
        # Add maze reward to utility table to track I've been here
        self.U_table_to_start[self.current_location] += self.maze_rewards[self.current_location]
        
        return next_location
    
    def next_move(self, sensors):
        self.times_visited[self.current_location] += 1
        
        # Get valid next locations
        visible_next_locations = self.get_visible_next_locations(sensors)

        # Get all valid next locations
        self.update_next_locations_table(visible_next_locations)
        
        if not self.found_goal:
            # Choose action with heuristic and negative rewards for visited locations
            next_location = self.choose_next_location()

            # Choose min visited location. Break ties with heuristic
            #next_location = self.choose_next_location_2()
            
            # Choose random min visited location
            #next_location = self.choose_next_location_3()
            
            # Choose random
            #next_location = self.choose_next_location_4()
            self.path_taken_to_goal.append(next_location)
        else:
            next_location = self.head_to_start()
            self.path_taken_to_start.append(next_location)
             
        rotation, movement = self.get_movements(next_location)

        # Update path taken and current location
        self.path_taken.append(next_location)
        self.current_location = next_location

        self.move_count += 1

        return rotation, movement