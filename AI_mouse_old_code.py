'''
Define a reward for each cell in the maze
Rewards are high in the center and decrease radially out

self.maze_rewards = np.zeros((maze_dim,maze_dim))
for x in [i for i in range(-maze_dim/2,maze_dim/2+1) if i != 0]:
    for y in [j for j in range(-maze_dim/2,maze_dim/2+1) if j != 0]:
        if x < 0: 
            dx = maze_dim/2 
        else: 
            dx = maze_dim/2 - 1
        if y < 0:
            dy = maze_dim/2
        else:
            dy = maze_dim/2 - 1
        self.maze_rewards[x+dx][y+dy] = maze_dim + 1 - (abs(x)+abs(y))
'''


       
'''
#### (2) Trémaux's algorithm ####
next_location = self.choose_path_less_travelled(valid_next_locations)
self.update_path_travelled(next_location)
'''

    def update_path_travelled(self,next_location):
        
        current_location = np.array(self.current_location)
        next_location = np.array(next_location)
        
        print "current location: {}".format(current_location)
        print "next_location: {}".format(next_location)
        
        step_dir = next_location - current_location
        step_dir = step_dir/max(abs(step_dir))
        
        path = next_location - step_dir
        while np.array_equal(path,current_location) is False:
            self.paths_travelled[tuple(path)] += 1
            path -= step_dir
			
			
    def choose_path_less_travelled(self,valid_locations):
        
        # Get travelledness of valid_locations
        valid_locations_travelledness = []
        for i in valid_locations:
            valid_locations_travelledness.append(self.paths_travelled[i])
        least_travelled = min(valid_locations_travelledness)

        # Make a list of all the equally less travelled paths
        paths_less_travelled = []
        for i in valid_locations:
            if self.paths_travelled[i] == least_travelled:
                paths_less_travelled.append(i)

        # Randomly choose from one of the less travelled paths
        next_location = random.choice(paths_less_travelled)
        
        return next_location
        
    def update_Q_values(self,maxQ):       
        for location in self.Q_table:
            for next_location in self.Q_table[location]:
               if next_location == self.current_location:
                    self.Q_table[location][next_location] += self.alpha*(self.maze_rewards[location]+self.gamma*maxQ) 