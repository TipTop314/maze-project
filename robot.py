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
        
        # Robot parameters
        self.learning = True
        self.epsilon = 0
        self.alpha = 0.5
        self.gamma = 0.1
        
        # Robot constants
        self.maze_dim = maze_dim

        # Learned information
        self.Q_table = dict()
        self.next_locations_table = dict()
        self.dead_ends = []

        # Present info
        self.move_count = 0
        self.current_location = start_location
        self.heading = start_heading

        # Past info
        self.path_taken = [start_location]
        self.last_action = (0,0)
        self.last_reward = 0
        self.last_location = start_location


        # Initialize utility table and maze rewards
        self.center_reward = +100
        self.not_center_reward = -1
        self.U_table = []
        self.maze_rewards = []
        self.initialize_tables()
        
        # tables for wandering
        self.wander_alpha = 0.1
        self.wander_locations = dict()
        self.wander_utility = np.zeros((self.maze_dim,self.maze_dim))
        self.wander_rewards = np.zeros((self.maze_dim,self.maze_dim))
        for x in [-1, 0]:
            for y in [-1,0]:
                self.wander_rewards[self.maze_dim/2 + x][self.maze_dim/2 + y] = +10
                
        
    def initialize_tables(self):
        # Initialize utility table with zero in the center
        # and with increasingly negative values as you move
        # radially out from the center
        self.U_table = np.zeros((self.maze_dim,self.maze_dim))
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
                self.U_table[x+dx][y+dy] = (-(abs(x)+abs(y)) + 2)*0.1
        
        # Large positive reward in the center and negative elsewhere
        self.maze_rewards = np.zeros((self.maze_dim,self.maze_dim))
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                self.maze_rewards[x][y] = self.not_center_reward
        for x in [-1, 0]:
            for y in [-1,0]:
                self.maze_rewards[self.maze_dim/2 + x][self.maze_dim/2 + y] = self.center_reward
        
    def get_visible_next_locations(self,sensors):
        valid_moves = []
        valid_moves = list(sensors)

        current_location = np.array(self.current_location)
        last_location = np.array(self.last_location)

        # # Get the steps and direction of the last move taken
        # last_move = current_location - last_location
        # if not sum(last_move)==0:
            # last_move_dir = last_move / np.linalg.norm(last_move)

            # # If robot's last move was to step forward, add stepping backward up
            # # to a maximum of the same number of steps to the valid moves list
            # if np.array_equal(last_move_dir,dir_move[self.heading]):
                # valid_moves.append(abs(sum(last_move)))

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

    def update_Q_table(self, visible_next_locations):
        ######################################
        # Create new entry in Q table
        # If new valid next locations were found, add to table
        # Update state and action
        ######################################

        # Add current location to the Q table if it's not there
        if not self.current_location in self.Q_table:       
            self.Q_table[self.current_location] = {x:0.01*self.maze_rewards[x] for x in visible_next_locations}
        else:
            # Loop through existing entries for valid next locations to see
            # if there are any that need to be added 
            locations_to_add = list(visible_next_locations)
            for i in visible_next_locations:
                for j in self.Q_table[self.current_location].keys():
                    if i[0] == j[0] and i[1] == j[1]:
                        locations_to_add.remove(i)
            
            # Add missing valid next locations to Q-table
            if not locations_to_add == []:
                for location in locations_to_add:
                    self.Q_table[self.current_location][location] = 0.01*self.maze_rewards[location]
        return

    def choose_next_location(self):
        
        # If current location is in table, choose from this list
        # If not, then current location is a dead end, either step
        # back in the direction you just came from (with get_backward_location)
        # or, if that direction is also a dead end, step backward one cell
        if self.current_location in self.next_locations_table:
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

        if self.learning == True and random.random() < 1 - self.epsilon:    
            # Make list of all next locations with maxU
            maxU_locations = []
            for location in next_locations:
                if self.U_table[location] == maxU:
                    maxU_locations.append(location)
            next_location = random.choice(maxU_locations)
        else:
            next_location = random.choice(self.next_locations_table[self.current_location])
            
        return next_location, maxU

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
        n_steps = 0
        current_location = np.array(self.current_location)
        last_location = np.array(self.last_location)

        # Get the steps and direction of the last move taken
        last_move = current_location - last_location
        if not sum(last_move)==0:
            last_move_dir = last_move / np.linalg.norm(last_move)
            # Check if last move was a forward step
            if np.array_equal(last_move_dir,dir_move[self.heading]):
                n_steps = abs(sum(last_move))
                dir_bwd_steps = np.array(dir_move[dir_reverse[self.heading]])
        
        # Convert number of steps to backward next locations
        backward_locations = []

        
        for x in range(1,n_steps+1):
            backward_locations.append(tuple((current_location + x*dir_bwd_steps)))
                
        return backward_locations

    def update_next_locations_table(self,visible_next_locations):
        
        # Add current location to the next_location_table if it's not there
        if not self.current_location in self.next_locations_table:       
            self.next_locations_table[self.current_location] = []
            
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
        del self.next_locations_table[self.current_location]
        
    def next_move(self, sensors):
        
        # Get valid next locations
        visible_next_locations = self.get_visible_next_locations(sensors)

        # Get all valid next locations
        self.update_next_locations_table(visible_next_locations)
    
        # Choose action and get maxU
        next_location, maxU = self.choose_next_location()
        
        # Update U_table value
        if self.move_count>0:
            #self.update_Q_values(maxQ)
            #self.Q_table[self.last_location][self.current_location] += self.alpha*(self.last_reward + self.gamma*maxQ) 
            self.U_table[self.current_location] += self.alpha*(self.maze_rewards[self.current_location])# + self.gamma*maxU)

        # Given action, get rotation, movement, new location, and new direction
        rotation, movement = self.get_movements(next_location)

        # Keep last location and reward for next time step
        self.path_taken.append(next_location)
        self.last_location = self.current_location
        self.last_reward = self.maze_rewards[self.current_location]
        self.current_location = next_location

        self.move_count += 1
        #self.epsilon = pow(0.99999,self.move_count)

        return rotation, movement
    
    def wander_setup(self):
    
        visited_locations = self.next_locations_table.keys() 
        for location in self.next_locations_table:
            self.wander_locations[location] = []
            for next_location in self.next_locations_table[location]:
                for place in visited_locations:
                    if next_location[0] == place[0] and next_location[1] == place[1]:
                        self.wander_locations[location].append(next_location)
        
        self.wander_locations[self.current_location] = [self.last_location]
        self.wander_locations[self.last_location].append(self.current_location)
        
    def wander(self, time_limit):
        for t in range(time_limit):
            # Update wander_utility
            maxU = -1
            for location in self.wander_locations[self.current_location]:
                if self.wander_utility[location] > maxU:
                    maxU = self.wander_utility[location]
            
            self.wander_utility[self.current_location] += self.wander_alpha*((self.wander_rewards[self.current_location])+self.gamma*maxU)
            
            # Randomly move to a new locations
            next_location = random.choice(self.wander_locations[self.current_location])
            self.last_location = self.current_location
            self.current_location = next_location