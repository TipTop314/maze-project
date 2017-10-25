# maze_project

AI_mouse_project.ipynb
- Development Python code. Use this notebook to replicate results from the report. Note that results will vary since the robot isn't deterministic.

robot.py
- The robot agent. This file contains all the agents discussed in the report:
  - The Explorer of the Unknown
  - The Directed Explorer
  - The Curious Explorer
- To run a test or a set of trials (using the AI_mouse_project notebook) for a specific explorer, you'll need to edit this file. The lines you'll need to comment/uncomment are in the next_move function. Uncomment the line starting with next_location under the explorers name to turn it on. Make sure all the other explorers which you aren't testing are commented out. You need to have at least one of the explorers under "if not self.found_goal" turned on. Just below that, there's also the option to turn on The Curious Explorer which will continue exploration after finding the goal, or keep that off and turn on "self.finished_exploring = True" to end the first run once the goal is found.

