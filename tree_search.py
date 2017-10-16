class PathGetter(object):
    
    def __init__(self,next_locations,start):
        self.all_paths = []
        self.loops = 0
        self.next_locations = next_locations
        self.start = start
        
    def print_tab(self,i):
        tab = ""
        for x in range(i):
            tab += "   "
        return tab
        
    def find_paths(self,path,i):
        
        self.loops += 1
        if self.loops>100:
            raise Exception
        
        # print all the next path directions
        branches = []
        for location in self.next_locations:
            if path[-1] in self.next_locations[location]:
                branches.append(location)
        print "{}{}{}".format(self.print_tab(i),"possible paths: ",branches)
        
        # continue building paths
        for location in self.next_locations:
            if path[-1] in self.next_locations[location]:
                new_path = list(path)
                new_path.append(location)
                
                print "{}{}".format(self.print_tab(i),new_path)
                
                if new_path[-1] == self.start:
                    self.all_paths.append(new_path)
                elif new_path[-1] in new_path[:-1]:
                    print "dead end"
                else:
                    self.find_paths(new_path,i+1)
        return


def choose_shortest_path(all_paths):
    path_lengths = [len(x) for x in all_paths]
    min_length = min(path_lengths)
    for i,val in enumerate(path_lengths):
        if val == min_length:
            return all_paths[i]        
    raise Exception