# PRRT: Python implementation for Rapidly-exploring Random Tree
<a href="http://www.youtube.com/watch?feature=player_embedded&v=wZBmb4afLGo
" target="_blank"><img src="http://img.youtube.com/vi/wZBmb4afLGo/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

PRRT is built on the ideas presented in:
- Blanco, Jose Luis, Mauro Bellone, and Antonio Gimenez-Fernandez. [TP-Space RRT-Kinematic Path Planning of Non-Holonomic Any-Shape Vehicles][TP-RRT] International Journal of Advanced Robotic Systems 12 (2015)
- [MRPT][] RRT implementation.

implementation is based on MRPT source code available at <https://github.com/MRPT/mrpt>

## Requirements:
- Python 3.x
- [numpy][]
- [matplotlib][]
- [SortedDict][]

Most scientific python distributions have these packages bundled already. This work is developed using [Anaconda][]
## How to:
**Q: How to understand the theory behind the code?**

A: See references [1] and [2]
___
**Q: How to understand the code?**

A: Start by reading the commented config files in the config directory. Then look at the implementation of 
*build_cpoints()* and *build_obstacle_grid()* in *CPTG* class in *prrt.ptg* module. From there
check the *solve()* in the *Planner* class in *prrt.planner* module. 
___
**Q: How to run PRRT?**

A: Make sure all required packages are installed then cd to PRRT directory and run:
```shell
    python planner_runner.py 1 .\config\planner.yaml
```
---
**Q: Where are the results?**

A: Once a solution is reached PRRT will dump a csv file with a trace of solution steps. The file name and location is configurable
by changing the 'csv_out_file' field in the 'planner.yaml' configuration file. The csv file has the following format:

|PTG Name| Node Id|x|y|theta(heading in deg)|phi(articulation angle in deg)|v|w|
|---|---|---|---|---|---|---|---|

Additionally, PRRT can plot solution as a series
of frame at each step in the solution. The location of the plots are configurable via the 'plot_solution' field in the 'planner.yaml'
configuration file.
___
**Q: How to change the map?**

A: Edit the planner .yaml file and change 'world_map_file' field. In the same file adjust
 world_width and world_height as needed. The map must be black and white. Black pixels are interpreted as occupied cells.
---  
**Q: What PTGs are implemented?**

A: Circular PTG and  AlphaA PTG

--- 
**Q: What vehicle models are implemented?**

A: Two vehicle models are now implemented:

    1.  ArticulatedVehicle : see references [3]
    2.  ArtculatedVehicleB : To Do: Add reference
ArticulatedVehicleB(default) is working for both forward and backward motion. ArticulateVehicle: is working for forward motion only.
___
**Q: How to implement a new vehicle model?**

A: In prrt.vehicle module create a class that inherits ArticulatedVehicle class. Take a look at class ArticulatedVehicleB for guidance.
___

## Performance:
Using the default configuration, on a i5-4210 laptop, the planner solve() was ran 25 times. Execution was terminated if by 5 minutes no solution was found.
 The results were:
 
 Solution found:  21 times. Not found : 4
 
 Average solution time (excluding not found) : 113 seconds
 
 Note on performance:
 The code is entirely written in python and run on a single thread. A direct port to c++ should give ~50x-100x reduction
  in execution time. Additionally, vectorising of some key functions (*invers_WS2TP*, *get_aptg_nearest_node*) using numpy
   should lead to significant reduction in run time.
 



## Runners:

Runner modules provide basic functionality for testing PRRT:
  - vehicle_runner.py : To test basic vehicle functions.
  - aptg_runner.py : To build APTGs (Articulated PTGs), trace trajectories and more.
  - planner_runner.py : Solve using prrt.

Running any runner without additional arguments will print help message with details on possible commands and their
 arguments.

For example:
In console, cd to prrt folder and run:
```shell
    python aptg_runner.py
```
will print:
```shell
PTG Runner!
Usage:
Run: python aptg_runner.py [command number] [arg1] [arg2] ....

Commands:
  1: Plot PTG cpoints
     Arguments:
       1: Vehicle configuration file
       2: APTG configuration file
       3: APTG initial articulation angle in deg
     Example: python aptg_runner.py 1 ./config/vehicle.yaml ./config/fwd_captg.yaml 20

  2: Trace vehicle trajectory at given initial phi(articulation angle) and alpha(steering angle).
     Arguments:
       1: Vehicle configuration file
        ...
        ...
        ...
        
```
## To Do:
* Currently *planner.solve()* terminates once it finds a valid trajectory connecting start pose with goal pose. It's better however, if the planner continued looking for other solutions and after some cut of time
return the best solution. Best solution can be in numbers of maneuvers, shortest time, shortest path or any other metric.
* Refactoring *build_cpoints()* from subclasses (e.g. *CPTG*) to super class *PTG*. Subclasses need to implement a 'control parameters (v,w) generation function' and the rest will be identical across PTGs.
* Support for Alpha-a PTG : current implementation needs some tweaking. From the result log the PTG is not currently being picked by the planner.
## Reference:
[1]  Blanco, Jose Luis, Mauro Bellone, and Antonio Gimenez-Fernandez. "TP-Space RRT-Kinematic Path Planning of Non-Holonomic Any-Shape Vehicles." International Journal of Advanced Robotic Systems 12 (2015).

[2]  Blanco, Jose-Luis, Javier González, and Juan-Antonio Fernández-Madrigal. "Extending obstacle avoidance methods through multiple parameter-space transformations." Autonomous Robots 24.1 (2008): 29-48.

[3]  Lamiraux, Florent, Sepanta Sekhavat, and J-P. Laumond. "Motion planning and control for Hilare pulling a trailer." IEEE Transactions on Robotics and Automation 15.4 (1999): 640-652.

[MRPT]: http://www.mrpt.org/tp-rrt
[TP-RRT]: http://cdn.intechopen.com/pdfs-wm/48420.pdf
[SortedDict]: http://www.grantjenks.com/docs/sortedcontainers/sorteddict.html
[matplotlib]: http://matplotlib.org/
[numpy]: http://www.numpy.org/
[Anaconda]: https://www.continuum.io/downloads

