# Maze Solving Agent

This was a project I completed as part of my Udacity Nanodegree. The project is based off the Micromouse competition, where teams design a robot to navigate a maze from the corner to the center without any other knowledge of the maze aside from what it can see around itself.

For this project, I created a virtual agent to navigate a similar type of maze. The agent navigates the maze twice, once to explore the maze, and then again to complete it as fast as possible. During the first run, the agent must find it's way to the goal, but can also choose to continue searching the maze afterwards. During the second run, the agent can use the information gained from the first run to plot the quickest path to the goal.

A more detailed description of the project can be found in the report above.

### AI_mouse_project.ipynb
Development Python code. Use this notebook to replicate results from the report. Note that results will vary since the robot isn't deterministic.

### robot.py
The robot agent. This file contains all the agents discussed in the report:
  - The Explorer of the Unknown
  - The Directed Explorer
  - The Curious Explorer
To run a test or a set of trials (using the AI_mouse_project notebook) for a specific explorer, you'll need to edit this file. The lines you'll need to comment/uncomment are in the next_move function. Uncomment the line starting with next_location under the explorers name to turn it on. Make sure all the other explorers which you aren't testing are commented out. You need to have at least one of the explorers under "if not self.found_goal" turned on. Just below that, there's also the option to turn on The Curious Explorer which will continue exploration after finding the goal, or keep that off and turn on "self.finished_exploring = True" to end the first run once the goal is found.

### visualizations.py
This contains all the helper functions I created for visualizing the maze in different ways.

### take_n_steps.py
This contains some code I used in conjunction with the AI_mouse_project notebook for running the robot through the maze

Libraries used for this project: numpy, pandas, matplotlib, random
