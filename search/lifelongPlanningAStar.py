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
# BENCHMARK = None

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
        self.next = None
        
        self.hasPath = False
        self.bestPath = None
        # self._last_path = None
        self.path = []
        self.popCount = 0
        self.changedEdges = list()  #new adder
          
        self.width = problem.getWalls().width
        self.height = problem.getWalls().height
        self.isPrimaryWall = problem.getPrimaryWalls()

        # init the start node
        self.g_rhsTuple = dict()
        for i in range(self.width):
            for j in range(self.height):
                self.g_rhsTuple[(i,j)] = [float("inf"),float("inf")]
        
        self.g_rhsTuple[self.start][1] = 0
        self.U.insert(self.start, self.calculateKey(self.start))  ##modify the k1, k2 in the U.insert

    def calculateKey(self, u):  #u: first time is the goal state(x,y)
        heuristic = self.h(u, self.goal) #manhattan distance
        g_rhsTuple = self.g_rhsTuple[u]
        return min(g_rhsTuple) + heuristic, min(g_rhsTuple)  #return keys = [k1,k2]

    def updateVertex(self, u, extraNode = None):
        if extraNode is None:
            extraNode = self.start

        # update rhs (if not start node)
        if u != extraNode:
            rhs_u = float("inf")  # if this node is a wall, the new rhs(s) is infinity
            if not self.isPrimaryWall[u[0]][u[1]]:
                neighbors = self.getNeighbors(u)
                for neighbor in neighbors:
                    # rhs_u = min(rhs_u, self.get_g_rhsTuple(neighbor)[0] + COST) #calculate the rhs(s) in the next step
                    rhs_u = min(rhs_u, self.g_rhsTuple[neighbor][0] + COST) #calculate the rhs(s) in the next step
            # self.set_g_rhsTuple(u, (BENCHMARK, rhs_u)) #update the rhs(s)
            self.g_rhsTuple[u][1] = rhs_u  #update the rhs(s)
            

        # remove from U
        self.U.removeU(u)

        # re-insert if locally underconsistent (inequality partially satisfied by upstream computeShortestPath)
        # g, rhs = self.get_g_rhsTuple(u)
        g, rhs = self.g_rhsTuple[u]
        if g != rhs:
            self.U.insert(u, self.calculateKey(u)) #U.insert is the function of CalcualteKey(u)

    def computeShortestPath(self):
        if self.hasPath:
            return  # don't try to re-compute an already-computed path, the PQ is empty

        # implicitly assumes start to goal
        g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
        topKeys = self.U.topKey()[1:3]
        calculateKey = self.calculateKey(self.goal)
        while self.keyComparision(topKeys, calculateKey) or (g_Sgoal != rhs_Sgoal):
            u = self.U.pop()
            self.popCount += 1
            g_u, rhs_u = self.g_rhsTuple[u]
            if g_u > rhs_u:
                # locally overconsistent
                self.g_rhsTuple[u][0] = self.g_rhsTuple[u][1]
                print('------local overconsistent----------', (self.g_rhsTuple[u]))
            else:
                self.g_rhsTuple[u][0] = float("inf")
                self.updateVertex(u)  # update the vertex itself
            for s in self.getNeighbors(u):
                self.updateVertex(s)  # update the successor vertices, in either case
                               
            # prep variables for next loop invariant test        
            if self.U.size() == 0:
                break  # all done! the whole graph is consistent
            g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
            calculateKey = self.calculateKey(self.goal)
            topKeys = self.U.topKey()[1:3]  
        
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
        return neighbors

    # def set_g_rhsTuple(self, u, tup):
    #     x, y = u        
    #     if tup[0] is not BENCHMARK:
    #         self.grid_costs[x][y][0] = tup[0]
    #     if tup[1] is not BENCHMARK:
    #         self.grid_costs[x][y][1] = tup[1]

    # def get_g_rhsTuple(self, u):
    #     x, y = u
    #     return self.grid_costs[x][y]
    
    # def nodeUpdate(self, u):
    #     x, y = u        
    #     self.changedEdges.append(u)

    #     self.isPrimaryWall[x][y] = True
    #     for neighbor in self.getNeighbors(u):
    #         self.changedEdges.append(neighbor)  # add all wall-adjacent edges to the queue to be update_vertex'd

    #     # process edge updates
    #     for changedEdge in self.changedEdges:
    #         self.updateVertex(changedEdge)
    #     self.changedEdges = list()  # we've updated all the edges
    #     print('4--------->')
        
    # def nextState(self):
    #     if self.next == self.goal:
    #         return self.next  # we're already at the goal; no need to move
        
    #     if self.next == None:
    #         self.next = self.start
    
    #     print('7----->', self.next)
    #     print('8----->', self.getNeighbors(self.next))
    #     tempdict = dict()
    #     templist = list()
    #     for neighbor in self.getNeighbors(self.next):
    #         tempdict[self.calculateKey(neighbor)] = neighbor
    #         templist.append(self.calculateKey(neighbor))
        
    #     print('---befor change list---', templist)
        
    #     templist.sort(key=lambda tup:tup[0])
    #     print('---after change list---', templist)
    #     for index in range(len(templist)-1):
    #         if templist[index][0] == templist[index+1][0] and templist[index][1] > templist[index+1][1]:
    #             templist[index], templist[index+1] = templist[index+1], templist[index]
        
    #     self.next = tempdict[templist[0]]
    #     if self.next == self.start:
    #         self.next = tempdict[templist[1]]
    #     self.path.append(self.next)
    #     # self.U.insert(self.next, self.calculateKey(self.next))
        
    #     print('9----->', self.next)
        
    #     return self.next
    
    # def getPath(self):
    #     temp = list(self.path)
    #     temp.append(self.start)
    #     return temp

    def make_wall_at(self, u):
        x, y = u
        if self.isPrimaryWall[x][y]:
            return

        # path might have changed!
        self.hasPath = False
        # self._last_path = self._best_path
        self.bestPath = None

        # "update the edge weights", or more accurately, update the adjacent, affected vertices
        self.isPrimaryWall[x][y] = True
        self.updateVertex(u)

    def extract_path(self, backward=True):
        if self.start == self.goal:
            return [self.start]  # trivial case

        self.computeShortestPath()  # if no shortest path is yet available, generate one
        if self.bestPath is None:
            # traverses the weights and returns a series of coordinates corresponding to the shortest path
            best_path = []

            if not backward:
                currPosition = self.start  # go from start to goal (D*lite)
                targetPosition = self.goal
            else:
                currPosition = self.goal  # go from goal to start (LPA*)
                targetPosition = self.start
            if self.g_rhsTuple[currPosition][0] == float("inf"):    
            # if self.get_g_rhsTuple(currPosition)[0] == float("inf"):
                return None  # no path between start and goal
            
            while currPosition != targetPosition:
                best_path.append(currPosition)
                currNeighbors = self.getNeighbors(currPosition)
                for i in range(len(currNeighbors)):
                    currNeighbors[i] = (currNeighbors[i], self.g_rhsTuple[currNeighbors[i]][1])
                    # currNeighbors[i] = (currNeighbors[i], self.get_g_rhsTuple(currNeighbors[i])[1])
                currNeighbors.sort(key=lambda tup: tup[1])
                currPosition = currNeighbors[0][0]  ## It should be tuple(x,y)

            best_path.append(currPosition)  # add the goal to the path
            self.bestPath = best_path
            if backward:
                self.bestPath.reverse()  #find the point which is the closest one to the start
        return self.bestPath
    
