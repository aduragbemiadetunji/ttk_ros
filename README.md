# ttk_ros
TTK22 ros project

Clone this repo into the source file where the package "exploration" files are.

The launch files are enough to demonstrate everything written in the report.


simulation 1: roslaunch gbplanner rmf_sim.launch gazebo_gui_en:=true world_file:=<path to world file>/virginia_mine.world
                roslaunch ttk_ros rmf_sim_1.launch

simulation 2: roslaunch ttk_ros rmf_sim_2.launch gazebo_gui_en:=true world_file:=<path to world file>/virginia_mine.world

simulation 3: roslaunch ttk_ros rmf_sim_3.launch gazebo_gui_en:=true world_file:=<path to world file>/virginia_mine.world
