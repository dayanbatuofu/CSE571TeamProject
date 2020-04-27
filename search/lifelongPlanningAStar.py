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
        self.U = util.PQueue()
        self.start = problem.getStartState()
        self.goal = problem.getGoalState()  #Add getGoalState in the searchAgent
#        self.next = None
        
        self.hasPath = False
        self.bestPath = None
        # self._last_path = None
        self.path = []
        self.popCount = 0
#        self.changedEdges = list()  #new adder
          
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
            rhslist = [float("inf")]
            if not self.isPrimaryWall[u[0]][u[1]]:
                neighbors = self.getNeighbors(u)                
                for neighbor in neighbors:
                    # rhs_u = min(rhs_u, self.get_g_rhsTuple(neighbor)[0] + COST) #calculate the rhs(s) in the next step
                    rhslist.append(min(rhs_u, self.g_rhsTuple[neighbor][0] + COST)) #calculate the rhs(s) in the next step
            # self.set_g_rhsTuple(u, (BENCHMARK, rhs_u)) #update the rhs(s)
            self.g_rhsTuple[u][1] = min(rhslist)  #update the rhs(s)
            

        # remove from U
        self.U.removeU(u)

        # re-insert if locally underconsistent (inequality partially satisfied by upstream computeShortestPath)
        # g, rhs = self.get_g_rhsTuple(u)
        g, rhs = self.g_rhsTuple[u]
        if g != rhs:
            self.U.insert(u, self.calculateKey(u)) #U.insert is the function of CalcualteKey(u)

    def computeShortestPath(self):
        if self.hasPath:
            return  # do nothing if the priority queue is already empty.

        # implicitly assumes start to goal
        g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
#        print('Before Values are ', (g_Sgoal, rhs_Sgoal))

        topKeys = self.U.topKey()
        calculateKey = self.calculateKey(self.goal)
        while self.keyComparision(topKeys, calculateKey) or (g_Sgoal != rhs_Sgoal):           
            u = self.U.pop()
            self.popCount += 1
            g_u, rhs_u = self.g_rhsTuple[u]
            if g_u > rhs_u:
                # locally overconsistent
                self.g_rhsTuple[u][0] = self.g_rhsTuple[u][1]   # Local Overconsistent
            else:
                self.g_rhsTuple[u][0] = float("inf")
                self.updateVertex(u)  # reupdate the vertex with g = inf
            for s in self.getNeighbors(u):
                self.updateVertex(s)  # update the successor vertices in both over and 
                                      # under consistent cases.
                               
            # Update the variables for next iteration in the loop       
            if self.U.size() == 0:
                break                 # if every node is consistent then break
            g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
            calculateKey = self.calculateKey(self.goal)
            topKeys = self.U.topKey()
        
#        print('After Values are ', (g_Sgoal, rhs_Sgoal))
        self.hasPath = True

    def keyComparision(self, tu, tgoal):
            
        if len(tu) == 2:
            k1_top, k2_top = tu
        elif len(tu) == 3:
            _, k1_top, k2_top = tu

        if len(tgoal) == 2:
            k1_goal, k2_goal = tgoal
        elif len(tgoal) == 3:
            _, k1_goal, k2_goal = tgoal
        
        if k1_top < k1_goal:
            return True         # k1 comparison check satisfied
        elif k1_top > k1_goal:
            return False        # k1 comparison check not satisfied
        else:
            return k2_top < k2_goal  # k2 check

    def getNeighbors(self, u):
        directions = [(u[0], u[1] + 1),  # north
                      (u[0] + 1, u[1]),  # east
                      (u[0], u[1] - 1),  # south
                      (u[0] - 1, u[1])]  # west
        neighbors = []

        neighbors = [(x,y) for (x,y) in directions if x in range(self.width) and y in range(self.height)]
        return neighbors


    def Update_Wall_Info(self, u):
        x, y = u
        if self.isPrimaryWall[x][y]:
            return

        # If wall is encountered then path might change, hence self.hasPath has to be set back to False
        self.hasPath = False
        self.bestPath = None

        # Recalculate edge costs of the adjacent vertices
        self.isPrimaryWall[x][y] = True
        self.updateVertex(u)

    def Find_Path(self):
        if self.start == self.goal:
            return [self.start]

        self.computeShortestPath()  # Compute the shortest path
        if self.bestPath is None:
            best_path = []

            currPosition = self.goal  # LPA*
            targetPosition = self.start
            if self.g_rhsTuple[currPosition][0] == float("inf"):    
                return None
            
            while currPosition != targetPosition:
                best_path.append(currPosition)
                currNeighbors = self.getNeighbors(currPosition)
                for i in range(len(currNeighbors)):
                    currNeighbors[i] = (currNeighbors[i], self.g_rhsTuple[currNeighbors[i]][1])
                currNeighbors.sort(key=lambda tup: tup[1])
                currPosition = currNeighbors[0][0]

            best_path.append(currPosition)  # goal location appended to the path
            self.bestPath = best_path
            self.bestPath.reverse()  #reverse the path
        return self.bestPath
    
