# PRRT: Python implementation for Rapidly-exploring Random Tree
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


## Usage:
To run or test the code use the provided runner modules:
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


[MRPT]: http://www.mrpt.org/tp-rrt
[TP-RRT]: http://cdn.intechopen.com/pdfs-wm/48420.pdf
[SortedDict]: http://www.grantjenks.com/docs/sortedcontainers/sorteddict.html
[matplotlib]: http://matplotlib.org/
[numpy]: http://www.numpy.org/
[Anaconda]: https://www.continuum.io/downloads
