# CSE571TeamProject(Life-long Planning for Path-Finding in Pacman Domain)

## TEAM NAME: Program and Control (PAC) - Group 5

## TEAM MEMBERS: Lei Zhang, Yi Chen, Siddhartha Cheruvu, Aaron Arul Maria John

## TOPIC CHOSEN: Topic 2 - Life-long planning

## GITHUB: https://github.com/dayanbatuofu/CSE571TeamProject

## Python Version: 2.7

## CONTRIBUTIONS OF EACH TEAM MEMBER:

	The  project  responsibilities  shared  by  each  team  memberare provided in this section.Siddhartha  and  
	
	Lei  were  in  charge  of  writing  the A* and D* Lite algorithms and also creating the priority queue 
	
	utility. Yi and  Aaron  were  in  charge  of  writing  and  debugging  the LPA* and Simple Replanning A* 
	
	codes. Lei and Yi preparedthe project report whereas Aaron and Siddhartha collected andorganized the data for visualization.


## INSTRUCTIONS TO RUN THE CODE:

This project is base on Pacman domain to implement different algorithms. Note that command `valgrind` should be installed before running memory test.

We added or modified the following files:

* `search\search.py`
* `search\searchAgent.py`
* `search\lifelongPlanningAStar.py`
* `search\dStarlite.py`
* `search\util.py`

* Copy all the submitted files into a folder.
* In the terminal, navigate the current directory to the above folder\search.
* Use the following commands to run different algorithms.

### Following commands are examples to run the different algorithms:

#### A*
`python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,prob=PositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

#### Simple Replanning A*

`python pacman.py -l mediumMaze -p SearchAgent -a fn=srastar,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

#### Lifelong Planning A*

`python pacman.py -l mediumMaze -p SearchAgent -a fn=lpastar,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

IMPORTANT NOTE: In the final visualization of the solution, only the optimal path from

    the start to goal states is shown. All the backtracking done is not visualized as the time
	
    taken to traverse all these back paths is significantly high.
	
    However, all the visited nodes are highlighted in red color in the final simulation for reference 


#### D* Lite

`python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

### Following commands are examples to obtain the memory usage and running time testing for different algorithms

`test python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`

`valgrind python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite,prob=UnknownPositionSearchProblem,heuristic=manhattanHeuristic --frameTime 0 -z .5`


We have created new layouts for checking the performance of the algorithms with varying complexity.
* `tinyMaze2.lay`
* `smallMaze_c1.lay`
* `smallMaze_c2.lay`
* `smallMaze_c3.lay`
* `smallMaze_c4.lay`
* `smallMaze_c5.lay`
* `hugeMaze.lay`

_c1 is the least complex (without any obstacles) whereas _c5 corresponds to highest complexity with lot of obstacles in the path.

Please use these layouts for checking the performance of algorithms in grid worlds of varying complexities.

For checking performance variation with maze size, please use the following layouts
* `tinyMaze2.lay`
* `smallMaze.lay`
* `mediumMaze.lay`
* `bigMaze.lay`
* `hugeMaze.lay`

For checking performance variation with maze complexity, please use the following layouts* `smallMaze_c1.lay`
* `smallMaze_c1.lay`
* `smallMaze_c2.lay`
* `smallMaze_c3.lay`
* `smallMaze_c4.lay`
* `smallMaze_c5.lay`
