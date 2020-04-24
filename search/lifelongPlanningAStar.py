# This code is for CSE 571 Team Project, contributors are as below:
# Siddhartha Cheruvu 
# Aaron Arul Maria John
# Yi Chen
# Lei Zhang
#
# This algorithm of LPA* is based on the following paper:
# "D* Lite"
# https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf


import util
from priorityQueue import Queue

# global variables
COST = 1
BENCHMARK = None

# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default

# noinspection PyAttributeOutsideInit
class LPAStar(object):
    def __init__(self, problem, Heuristic=util.manhattanDistance):
        self.h = Heuristic  # expects a lambda that can be called
        self.U = Queue()
        self.start = problem.getStartState()
        self.goal = problem.getGoalState()  #Add getGoalState in the searchAgent
        
        self.hasPath = False
        self.bestPath = None
        # self._last_path = None
        self.popCount = 0
        self.changedEdges = list()  #new adder

        # set up the map/grid
        x_grid, y_grid = problem.getWalls().width, problem.getWalls().height
        self.width = x_grid
        self.height = y_grid
        self.grid_costs = [[[float("inf"), float("inf")] for j in range(y_grid)] for j in range(x_grid)]
        self.hitwall = problem.getWalls()

        # init the start node
        self.set_g_rhsTuple(self.start, (BENCHMARK, 0)) #Algorithm step 05
        self.U.insert(self.start, self.calculateKey(self.start))  ##modify the k1, k2 in the U.insert

    def calculateKey(self, u):  #u: first time is the goal state(x,y)
        heuristic = self.h(u, self.goal) #manhattan distance
        g_rhsTuple = (float("inf"), 0) if u == self.start else self.get_g_rhsTuple(u)
        # if u == self.start:
            # g_rhsTuple = (float("inf"), 0)
        # else:
            # g_rhsTuple = self.get_g_rhsTuple(u) # obtain the rhs(s) when s is the goal
        return min(g_rhsTuple) + heuristic, min(g_rhsTuple)  #return keys = [k1,k2]

    def updateVertex(self, u, extraNode = None):
        if extraNode is None:
            extraNode = self.start

        # update rhs (if not start node)
        if u != extraNode:
            rhs_u = float("inf")  # if this node is a wall, the new rhs(s) is infinity
            if not self.hitwall[u[0]][u[1]]:
                neighbors = self.getNeighbors(u)
                for neighbor in neighbors:
                    rhs_u = min(rhs_u, self.get_g_rhsTuple(neighbor)[0] + COST) #calculate the rhs(s) in the next step
            self.set_g_rhsTuple(u, (BENCHMARK, rhs_u)) #update the rhs(s)

        # remove from U
        self.U.removeU(u)

        # re-insert if locally underconsistent (inequality partially satisfied by upstream computeShortestPath)
        g, rhs = self.get_g_rhsTuple(u)
        if g != rhs:
            self.U.insert(u, self.calculateKey(u)) #U.insert is the function of CalcualteKey(u)

    def computeShortestPath(self):
        if self.hasPath:
            return  # don't try to re-compute an already-computed path, the PQ is empty

        # implicitly assumes start to goal
        g_Sgoal, rhs_Sgoal = self.get_g_rhsTuple(self.goal) #rhs(goal) != g(goal)
        # goal_keys = self.calculateKey(self.goal)
        # top_keys = self.U.topKey()[1:3]  # slice the peek to get the priority tuple only
        # while (g_Sgoal != rhs_Sgoal) or (self.keyComparision(top_keys, goal_keys)):
        while (self.U.size() > 0 and self.keyComparision(self.U.topKey(), self.calculateKey(self.goal))) or (g_Sgoal != rhs_Sgoal):
            u = self.U.pop()
            self.popCount += 1
            g_u, rhs_u = self.get_g_rhsTuple(u)  # pull these again; they may be different than when it was pushed
            if g_u > rhs_u:
                # locally overconsistent
                self.set_g_rhsTuple(u, (rhs_u, BENCHMARK))  # g(s) = rhs(s)
            else:
                self.set_g_rhsTuple(u, (float("inf"), BENCHMARK))  # g(s) = infinity
                self.updateVertex(u)  # update the vertex itself
            for s in self.getNeighbors(u):
                self.updateVertex(s)  # update the successor vertices, in either case

        self.hasPath = True

    def keyComparision(self, tup1, tup2):
        if not isinstance(tup1, tuple):
            raise ValueError("Left-side tuple is not correct key tuple: {}".format(tup1))

        if not isinstance(tup2, tuple):
            raise ValueError("Right-side tuple is not correct key tuple: {}".format(tup2))
            
        if len(tup1) == 2:
            k1_top, k2_top = tup1
        elif len(tup1) == 3:
            _, k1_top, k2_top = tup1
        else:
            raise ValueError("Left-side tuple contains unexpected arity: {0}".format(tup1))

        if len(tup2) == 2:
            k1_goal, k2_goal = tup2
        elif len(tup2) == 3:
            _, k1_goal, k2_goal = tup2
        else:
            raise ValueError("Right-side tuple contains unexpected arity: {0}".format(tup2))
        
        if k1_top < k1_goal:
            return True  # first primary wins
        elif k1_top > k1_goal:
            return False  # second primary wins
        else:
            return k2_top < k2_goal  # secondaries break tied primaries

    def getNeighbors(self, u):
        directions = [(u[0], u[1] + 1),  # north
                      (u[0] + 1, u[1]),  # east
                      (u[0], u[1] - 1),  # south
                      (u[0] - 1, u[1])]  # west
        neighbors = []

        neighbors = [(x,y) for (x,y) in directions if x in range(self.width) and y in range(self.height)]
        # for direction in directions:
        #     x, y = direction
        #     if x in range(self.width-1) and y in range(self.height-1):
        #         neighbors.append(direction)
        return neighbors

    def set_g_rhsTuple(self, u, tup):
        x, y = u        
        if tup[0] is not BENCHMARK:
            self.grid_costs[x][y][0] = tup[0]
        if tup[1] is not BENCHMARK:
            self.grid_costs[x][y][1] = tup[1]

    def get_g_rhsTuple(self, u):
        x, y = u
        return self.grid_costs[x][y]

    #TODO change later
    def make_wall_at(self, u):
        x, y = u

        if self.hitwall[x][y]:
            return

        # path might have changed!
        self.hasPath = False
        # self._last_path = self._best_path
        self.bestPath = None

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        self.hitwall[x][y] = True
        self.updateVertex(u)

    def extract_path(self, backward=True):
        if self.start == self.goal:
            return [self.start]  # trivial case

        self.computeShortestPath()  # if no shortest path is yet available, generate one
        if self.bestPath is None:
            # traverses the weights and returns a series of coordinates corresponding to the shortest path
            best_path = []

            # if not backward:
            #     curr_pos = self.start  # go from start to goal (D*lite)
            #     target_pos = self._goal
            # else:
            currPosition = self.goal  # go from goal to start (LPA*)
            targetPosition = self.start
            if self.get_g_rhsTuple(currPosition)[0] == float("inf"):
                return None  # no path between start and goal

            while currPosition != targetPosition:
                best_path.append(currPosition)
                currNeighbors = self.getNeighbors(currPosition)
                for i in range(len(currNeighbors)):
                    currNeighbors[i] = (currNeighbors[i], self.get_g_rhsTuple(currNeighbors[i])[1])
                currNeighbors.sort(key=lambda tup: tup[1])
                currPosition = currNeighbors[0][0]  ## It should be tuple(x,y)

            best_path.append(currPosition)  # add the goal to the path
            self.bestPath = best_path
            if backward:
                self.bestPath.reverse()  #find the point which is the closest one to the start
        return self.bestPath
    
        #TODO Later
    
        def nodeUpdate(self, u):
            x, y = u
            
            # startNeighbors = self.getNeighbors(self.start)
            
            # if u not in startNeighbors:
            #     raise ValueError("A wall cannot be discovered at a non-adjacent location; this breaks LPA.")
            self.changedEdges.append(u)
    
            self.hitwall[x][y] = True
            for neighbor in self.getNeighbors(u):
                self.changedEdges.append(neighbor)  # add all wall-adjacent edges to the queue to be update_vertex'd
    
            # process edge updates
            for changedEdge in self.changedEdges:
                # self.updateVertex(changedEdge, self.goal)
                self.updateVertex(changedEdge)
            self.changedEdges = list()  # we've updated all the edges
            print('4--------->')