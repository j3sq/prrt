# PRRT: Python implementation for Rapidly-exploring Random Tree
PRRT is built on the ideas presented in:
- Blanco, Jose Luis, Mauro Bellone, and Antonio Gimenez-Fernandez. [TP-Space RRT-Kinematic Path Planning of Non-Holonomic Any-Shape Vehicles][TP-RRT] International Journal of Advanced Robotic Systems 12 (2015)
- [MRPT][] RRT implementation.

implementation is based on MRPT source code avaiable at <https://github.com/MRPT/mrpt>

## Requirments:
- Python 3.x
- [numpy][]
- [matplotlib][]
- [SortedDict][]
Most scintific python distributions have these packages bundled already. This work is developed using [Anaconda][] 


## Usage:
A working example is supplied in \_\_main\_\_

In console, cd to prrt folder and run:
```python
    python __main__.py
```


[MRPT]: http://www.mrpt.org/tp-rrt
[TP-RRT]: http://cdn.intechopen.com/pdfs-wm/48420.pdf
[SortedDict]: http://www.grantjenks.com/docs/sortedcontainers/sorteddict.html
[matplotlib]: http://matplotlib.org/
[numpy]: http://www.numpy.org/
[Anaconda]: https://www.continuum.io/downloads
