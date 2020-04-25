# CSE571TeamProject


Below code should be modified:

dStar.py -- this part has already been done. Please check the code based on algorithm. If you want to get the result, please run the command: 

`python pacman.py -l mediumMaze -p SearchAgent -a fn=dstarlite, prob=UnknownPositionSearchProblem, heuristic=manhattanHeuristic --frameTime 0 -z .5`

lifelongPlanningAStar.py -- there are some `TODO` part at the end of code to make `LPA*` run smoothly. Please check the code based on algorithm. If you want to get the result, please run the command:  

`python pacman.py -l mediumMaze -p SearchAgent -a fn=lpastar, prob=UnknownPositionSearchProblem, heuristic=manhattanHeuristic --frameTime 0 -z .5`

search.py -- main structure of D* and simply replanning with A* baseline is ready. main structure of `LPA*` should be discussed how to write. I put some reference in the main structure of `LPA*`, it may be useful. If you want to get the result of simply replanning with A* baseline, please run the command: 

`python pacman.py -l mediumMaze -p SearchAgent -a fn=srastar, prob=UnknownPositionSearchProblem, heuristic=manhattanHeuristic --frameTime 0 -z .5`

searchAgent.py -- I write a new Class named `UnknownPositionSearchProblem` based on reference code. The reason I use a new Class is that we should firstly set an initial grid world without any information. It means that we don't know the wall in the grid world at the beginning time. Then we could use D* lite, LPA* and simple replanning with A* to help the pacman to find the optimal path. The important part in Class `UnknownPositionSearchProblem` is that I set `self.wall[x][y] = False` based on reference code. This setting means that the pacman don't know where is the wall.

priorityQueue.py -- this part is done. You could change structure as you want.
