# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Actions
import lifelongPlanningAStar as lpa
import dStar as dsl
# import d_star_lite_hans as dsl
from game import Directions

N = Directions.NORTH
S = Directions.SOUTH
E = Directions.EAST
W = Directions.WEST

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    frontier = util.Stack()
    explored = []
    
    if problem.getStartState() != None:
        frontier.push((problem.getStartState(),[]))
        explored.append(problem.getStartState())
    else:
        return '{}--->{}'.format('Start state is None','Please check the initial state')
    
    if problem.isGoalState(problem.getStartState()):
        return '{}--->{}'.format('Stop the game','Goal has been found')
    
    while not frontier.isEmpty():
        parent = frontier.pop() 
        explored.append(parent[0]) #add the parent node's location in the explored set
        direction = parent[1] #obtain the parent node's direction
        
        if problem.isGoalState(parent[0]):
            return direction

        for child in problem.getSuccessors(parent[0]):
            next_direction = direction[:]
            
            if child[0] not in explored:
                next_direction.append(child[1])
                frontier.push((child[0], next_direction))
                
    return next_direction

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    frontier = util.Queue()
    explored = []
    frontier_store_node = []
    
    if problem.getStartState() != None:
        frontier.push((problem.getStartState(),[]))
        explored.append(problem.getStartState())
        frontier_store_node.append(problem.getStartState())
    else:
        return '{}--->{}'.format('Start state is None','Please check the initial state')
    
    if problem.isGoalState(problem.getStartState()):
        return '{}--->{}'.format('Stop the game','Goal has been found')

    while not frontier.isEmpty():
        parent = frontier.pop() 
        explored.append(parent[0]) #add the parent node's location in the explored set
        direction = parent[1] #obtain the parent node's direction
        
        if (problem.isGoalState(parent[0])):
            return direction

        for child in problem.getSuccessors(parent[0]):
            next_direction = direction[:]
            
            if child[0] not in (explored and frontier_store_node):
                next_direction.append(child[1])
                frontier.push((child[0], next_direction))
                frontier_store_node.append(child[0])
                
    return direction

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    explored = []
    frontier_store_node = []

    if problem.getStartState() != None:
        frontier.push((problem.getStartState(),[]), 0)
        frontier_store_node.append(problem.getStartState())
    else:
        return '{}--->{}'.format('Start state is None','Please check the initial state')
    
    if problem.isGoalState(problem.getStartState()):
        return '{}--->{}'.format('Stop the game','Goal has been found')
    
    while not frontier.isEmpty():
        parent = frontier.pop()
        direction = parent[1]
        
        if problem.isGoalState(parent[0]):
            return direction
        
        if parent[0] not in explored:           
            for child in problem.getSuccessors(parent[0]):
                if child[0] not in explored:
                    next_direction = direction[:]
                    next_direction.append(child[1])
                    frontier.push((child[0], next_direction), problem.getCostOfActions(next_direction))
                    frontier_store_node.append(child[0])
        
        explored.append(parent[0])
                                 
    return direction


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    explored = []
    frontier_store_node = []
    
    if problem.getStartState() != None:
        frontier.push((problem.getStartState(), []), 0)
        frontier_store_node.append(problem.getStartState())
    else:
        return '{}--->{}'.format('Start state is None','Please check the initial state')
    
    if problem.isGoalState(problem.getStartState()):
        return '{}--->{}'.format('Stop the game','Goal has been found')
    
    while not frontier.isEmpty():
        parent = frontier.pop()
        direction = parent[1]
        
        if problem.isGoalState(parent[0]):
            return direction
        
        if parent[0] not in explored:
            for child in problem.getSuccessors(parent[0]):
                if child[0] not in explored:
                    next_direction = direction[:]
                    next_direction.append(child[1])
                    frontier.push((child[0], next_direction), heuristic(child[0], problem) + problem.getCostOfActions(next_direction))
                    frontier_store_node.append(child[0])
                    
        explored.append(parent[0])
                          
    return direction
    

def getDirection(currNode, nextNode):
    curr_x, curr_y = currNode
    next_x, next_y = nextNode
    if curr_y == next_y:
        if curr_x < next_x:
            return Directions.EAST
        if curr_x > next_x:
            return Directions.WEST
    if curr_x == next_x:
        if curr_y < next_y:
            return Directions.NORTH
        if curr_y > next_y:
            return Directions.SOUTH

def simpleReplanningAStarSearch(problem, heuristic):
    """
    Applies AStarSearch in the scenario where the agent only knows the goal
    state and does not know the location of the walls.  When the agent finds out
    a location of a wall from its successor states, it will need to restart the
    AStarSearch.

    We can initally test the implementation of this algorithm with tinyMaze grid
    using this command:
    python pacman.py -l tinyMaze -p SearchAgent -a fn=nrastar,prob=ReplanningSearchProblem,heuristic=manhattanHeuristic

    Then we can verify that the algorithm works with other grids, using the
    layouts from the layouts/ directory.
    """
    startState = problem.getStartState()
    x, y = startState[0], startState[1]
    explored = []
    directionList = aStarSearch(problem, heuristic)     # Get Optimal Path for grid without the knowledge
                                                        # of the inner walls.
#    print('---list---', directionList)
    while (x, y) != problem.getGoalState():
        #----------loop start----------
        explored.append((x, y))
        dx, dy = Actions.directionToVector(directionList.pop(0))
        next_x, next_y = int(x + dx), int(y + dy)
#        print('1--->', (next_x, next_y))

        neighborDirections = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        for neighborDirection in neighborDirections:
            dx, dy = Actions.directionToVector(neighborDirection)
            nbrx, nbry = int(x + dx), int(y + dy)
            # Here we check whether the neighbor location has a wall or not
            existObstacle = not problem.isPrimaryWalls(nbrx, nbry) and problem.isWall(nbrx, nbry)
            if existObstacle:
                problem.setPrimaryWalls(nbrx, nbry)        
    
        # Now that we have checked all the neighbors, replanning is done if needed i.e, when
        # there is an internal wall encountered at the next grid.
        if problem.isPrimaryWalls(next_x, next_y):  #if next state is the obstacle
            directionList = aStarSearch(problem, heuristic)
            dx, dy = Actions.directionToVector(directionList.pop(0))
            next_x, next_y = int(x + dx), int(y + dy)       
    
        x, y = next_x, next_y           # Update the x,y location to the next grid
        problem.setStartState(x, y)     # Since Simple Replanning A* recalculates every time it encounters
                                        # a wall, the problem's start state is reset everytime


    explored.append((x, y))
    directions = []
    directions = [getDirection(explored[i], explored[i+1]) for i in range(len(explored)-1)]
    problem.setStartState(startState[0], startState[1])  # reset the start state, for accurate path cost eval
    return directions


def lpaStarSearch(problem):  
    # This is the main procedure of the LPA* algorithm. Its structured is created
    # based on the pseudo code provided in the reference "https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf"    
    
    lpaStar = lpa.LPAStar(problem)
    startState = problem.getStartState()
    explored = []
    nodeList = lpaStar.Find_Path()[1:]  # compute the shortest path from the start location
    
    while startState != problem.getGoalState():     # Until the terminal state is reached
        explored.append(startState)
        nextNode = nodeList.pop(0)             
        neighborDirections = [N,S,E,W]
        for neighborDirection in neighborDirections:    # For all the successors update the wall information
            dx, dy = Actions.directionToVector(neighborDirection)
            next_x, next_y = int(startState[0] + dx), int(startState[1] + dy)
            if problem.isWall(next_x, next_y):
                lpaStar.Update_Wall_Info((next_x, next_y))
            
        if lpaStar.hasPath is False:  # When agent meet the obstacle
            # need to replan
            newnodeList = lpaStar.Find_Path()
            if nextNode not in newnodeList:
                # This is the point where the path has to be back tracked              
                valid_path = list(explored)
                explored.pop()  # remove the current position
                newtrace = explored.pop() ## Find the point before the current postion
                # go back to the divergence point
                while newtrace not in newnodeList:
                    newtrace = explored.pop()

                # pop until we reach the intersection point
                while newnodeList.pop(0) != newtrace:
                    continue
                
                setindex = 0
                while setindex == 0:
                    lastpop = valid_path.pop()
                    if lastpop == newtrace:
                        setindex = 1
                # Get the corrected path
                explored = valid_path + [newtrace]
                
                nodeList = newnodeList
                nextNode = nodeList.pop(0)
                
            else:
                while nextNode != newnodeList.pop(0):
                    continue  # pop until they match, then continue with the updated path
                nodeList = newnodeList
        startState = nextNode

    explored.append(startState)
    directions = []
    directions = [getDirection(explored[i], explored[i+1]) for i in range(len(explored)-1)]
    # specify the original start state, to show correct path cost eval
    problem.setStartState = (startState[0], startState[1])  
    problem._expanded = lpaStar.popCount
    return directions

def dStarLiteSearch(problem):
    startState = problem.getStartState()
    x, y = startState[0], startState[1]
    dStarLite = dsl.DStarLite(problem)
    print('2----->', (x,y))
    print('3-------->',(dStarLite.width, dStarLite.height))
    while (x, y) != problem.getGoalState():
        x, y = dStarLite.findNewStart()  # find the new start
        neighborDirections = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        for neighborDirection in neighborDirections:
            dx, dy = Actions.directionToVector(neighborDirection)
            nextx, nexty = int(x + dx), int(y + dy)
            print('5------->', (nextx, nexty))
            print('6------->', problem.isWall(nextx, nexty))
            if problem.isWall(nextx, nexty):  #edge costs have changed
                dStarLite.nodeUpdate((nextx, nexty))  
        # x, y = dStarLite.findNewStart()  # find the new start
        print('10~~~~~~~~~~~~~~~~~~')
    path = dStarLite.getPath()
    directions = []
    directions = [getDirection(path[i], path[i+1]) for i in range(len(path)-1)]
    problem._expanded = dStarLite.popCount
    print(directions)
    return directions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
srastar = simpleReplanningAStarSearch
lpastar = lpaStarSearch
dstarlite = dStarLiteSearch