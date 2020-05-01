# CSE571TeamProject(Life-long Planning for Path-Finding in Pacman Domain)

This project is base on Pacman domain to implement different algorithms. Note that command `valgrind` should be installed before running memory test.

We add or modify these following code:

`search\seach.py`
`search\searchAgent.py`
`search\lifelongPlanningAStar.py`
`search\dStarLite.py`
`search\util.py`

##Following command will show the example to run the different algorithms:

###A*
`python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,prob=PositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

###Simple Replanning A*

`python pacman.py -l mediumMaze -p SearchAgent -a fn=srastar,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

###Lifelong Planning A*

`python pacman.py -l mediumMaze -p SearchAgent -a fn=lpastar,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

###D* Lite

Following command will show the example to obtain the memory and running time testing for different 

`test python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

`valgrind python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`
