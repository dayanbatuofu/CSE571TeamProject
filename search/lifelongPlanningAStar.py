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

COST = 1

class LPAStar(object):
    def __init__(self, problem, Heuristic=util.manhattanDistance):
        self.h = Heuristic
        self.U = util.PQueue()
        self.start = problem.getStartState()
        self.goal = problem.getGoalState()
#        self.next = None
        
        self.hasPath = False
        self.bestPath = None
        self.path = []
        self.popCount = 0
          
        self.width = problem.getWalls().width
        self.height = problem.getWalls().height
        self.isPrimaryWall = problem.getPrimaryWalls()

        self.g_rhsTuple = dict()
        for i in range(self.width):
            for j in range(self.height):
                self.g_rhsTuple[(i,j)] = [float("inf"),float("inf")]
        
        self.g_rhsTuple[self.start][1] = 0
        self.U.insert(self.start, self.calculateKey(self.start))    # Update the U list

    def calculateKey(self, u):
        heuristic = self.h(u, self.goal) # manhattan distance is considered to be the heuristic
        g_rhsTuple = self.g_rhsTuple[u]
        return min(g_rhsTuple) + heuristic, min(g_rhsTuple)  #return keys = [k1,k2]

    def updateVertex(self, u, extraNode = None):
        if extraNode is None:
            extraNode = self.start

        if u != extraNode:
            rhs_u = float("inf")
            rhslist = [float("inf")]
            if not self.isPrimaryWall[u[0]][u[1]]:
                neighbors = self.getNeighbors(u)                
                for neighbor in neighbors:
                    rhslist.append(min(rhs_u, self.g_rhsTuple[neighbor][0] + COST))
            self.g_rhsTuple[u][1] = min(rhslist)  #update the rhs value
            

        self.U.removeU(u)

        g, rhs = self.g_rhsTuple[u]
        if g != rhs:
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self):
        if self.hasPath:
            return  # If the priority queue is empty, return nothing.

        g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
#        print('Before Values are ', (g_Sgoal, rhs_Sgoal))

        while (self.U.size() > 0 and self.keyComparision(self.U.topKey(), self.calculateKey(self.goal))) or (g_Sgoal != rhs_Sgoal):           
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
             # if every node is consistent then break
            g_Sgoal, rhs_Sgoal = self.g_rhsTuple[self.goal]
        
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
    
