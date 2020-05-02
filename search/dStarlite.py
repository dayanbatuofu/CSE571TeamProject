# This code is for CSE 571 Team Project, contributors are as below:
# Siddhartha Cheruvu 
# Aaron Arul Maria John
# Yi Chen
# Lei Zhang
#
# This algorithm of LPA* is based on the following paper:
# "D* Lite"
# https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf

import lifelongPlanningAStar as lpa
import util

# global variables
COST = 1

class DStarLite(lpa.LPAStar):
    def __init__(self, problem, Heuristic=util.manhattanDistance):
        # init the containers
        self.h = Heuristic  
        self.U = util.PQueue()
        self.km = 0  
        self.start = problem.getStartState()
        self.goal = problem.getGoalState()  
        self.last = problem.getStartState()
        self.path = []
        self.changedEdges = list()
        self.popCount = 0

        self.width = problem.getWalls().width
        self.height = problem.getWalls().height
        self.isPrimaryWall = problem.getPrimaryWalls()

        # Goal node initiation
        self.g_rhsTuple = dict()
        for i in range(self.width):
            for j in range(self.height):
                self.g_rhsTuple[(i,j)] = [float("inf"),float("inf")]

        self.g_rhsTuple[self.goal][1] = 0
        self.U.insert(self.goal, self.calculateKey(self.goal))  ## Update local inconsistency list
        
        self.computeShortestPath()


    def calculateKey(self, u):
        heuristic = self.h(u, self.start) # manhattan distance is used as heuristic
        g_rhsTuple = self.g_rhsTuple[u]
        return min(g_rhsTuple) + heuristic + self.km, min(g_rhsTuple)  #return keys = [k1,k2]

    
    def updateVertex(self, u, extraNode = None):
        if extraNode is None:
            extraNode = self.start

        if u != extraNode:
            rhs_u = float("inf")  # if the node is a wall, set rhs(s) to infinity
            rhslist = [float("inf")]
            if not self.isPrimaryWall[u[0]][u[1]]:
                neighbors = self.getNeighbors(u)
                for neighbor in neighbors:
                    rhslist.append(min(rhs_u, self.g_rhsTuple[neighbor][0] + COST)) # calculate the rhs(s) in the next step
            self.g_rhsTuple[u][1] = min(rhslist)  #update the rhs value
            

        self.U.removeU(u)

        g, rhs = self.g_rhsTuple[u]
        if g != rhs:
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self):

        g_Sstart, rhs_Sstart = self.g_rhsTuple[self.start]
        while (self.U.size() > 0 and self.keyComparision(self.U.topKey(), self.calculateKey(self.start))) or g_Sstart != rhs_Sstart:
            k_old = self.U.topKey()
            u = self.U.pop()
            self.popCount += 1
            g_u, rhs_u = self.g_rhsTuple[u]
            if self.keyComparision(k_old, self.calculateKey(u)):
                self.U.insert(u, self.calculateKey(u))
            elif g_u > rhs_u:
                # locally overconsistent
                self.g_rhsTuple[u][0] = self.g_rhsTuple[u][1] 
#                print('------local overconsistent----------')
            else:
                self.g_rhsTuple[u][0] = float("inf")
                self.updateVertex(u, self.goal)
#                print('------local underconsistent----------')
            for s in self.getNeighbors(u):
                self.updateVertex(s, self.goal) 
            g_Sstart, rhs_Sstart = self.g_rhsTuple[self.start] 
#            print('1------->', (g_Sstart, rhs_Sstart))
            
        self.hasPath = True

    def nodeUpdate(self, u):
        x, y = u
        
        startNeighbors = self.getNeighbors(self.start)
        
        if u not in startNeighbors:
            raise ValueError("A wall cannot be discovered at a non-adjacent location; this breaks D* Lite.")
        self.changedEdges.append(u)

        self.isPrimaryWall[x][y] = True
        for neighbor in self.getNeighbors(u):
            self.changedEdges.append(neighbor)

        # process edge updates
        self.km += self.h(self.last, self.start)
        self.last = self.start
        for changedEdge in self.changedEdges:
            self.updateVertex(changedEdge, self.goal)
        self.changedEdges = list()
        self.computeShortestPath()
#        print('4--------->')

    def findNewStart(self):
        if self.start == self.goal:
            return self.start

        g_Sstart, rhs_Sstart = self.g_rhsTuple[self.start]
        if g_Sstart == float("inf"):
            return self.start

        argmin = (None, float("inf"))
        for neighbor in self.getNeighbors(self.start):
            weight = COST + self.g_rhsTuple[neighbor][0]
            if weight < argmin[1]:
                argmin = (neighbor, weight)   
        self.path.append(self.start)
        self.start = argmin[0]
        
#        print('end----->', argmin)   
#        print('9----->', self.start)
        
        return self.start
    
    def getPath(self):
        temp = list(self.path)
        temp.append(self.start)
        return temp
    

    
