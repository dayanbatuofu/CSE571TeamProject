# This code is for CSE 571 Team Project, contributors are as below:
# Siddhartha Cheruvu 
# Aaron Arul Maria John
# Yi Chen
# Lei Zhang
#
# This algorithm of LPA* is based on the following paper:
# "D* Lite"
# https://www.aaai.org/Papers/AAAI/2002/AAAI02-072.pdf

from priorityQueue import Queue
import lifelongPlanningAStar as lpa
import util

# global variables
COST = 1
BENCHMARK = None


# assumptions:
# all edge weights are either 1 (adjacent) or infinity (non-adjacent or walls)
# 2-dimensional, rectilinear map; all squares which are adjacent are connected by default


# noinspection PyAttributeOutsideInit
class DStarLite(lpa.LPAStar):
    def __init__(self, problem, Heuristic=util.manhattanDistance):
        # init the containers
        self.h = Heuristic  # expects a lambda that can be called
        self.U = Queue()
        self.km = 0  # used to offset the DPQ, to reduce rebalancing
        self.start = problem.getStartState()
        self.goal = problem.getGoalState()  #Add getGoalState in the searchAgent
        self.last = problem.getStartState()
        self.path = []
        self.changedEdges = list()
        self.popCount = 0

        self.hasPath = False
        self.bestPath = None
        self.lastPath = None

        # set up the map/grid
        x_grid, y_grid = problem.getWalls().height, problem.getWalls().width
        self.width = x_grid
        self.height = y_grid
        self.grid_costs = [[[float("inf"), float("inf")] for i in range(x_grid)] for j in range(y_grid)]
        self.hitwall = problem.getWalls()

        # init the goal node (works backward)
        print('~~~~~~~~~~~~~~~~~')
        print(1, self.start)
        self.set_g_rhsTuple(self.start, (BENCHMARK, 0)) #Algorithm step 05
        self.U.insert(self.start, self.calculateKey(self.start))  ##modify the k1, k2 in the U.insert

        # find the path at least once, so we can take a step if needed
        self.computeShortestPath()

    def calculateKey(self, u):
        heuristic = self.h(u, self.goal) #manhattan distance
        g_rhsTuple = (float("inf"), 0) if u == self.start else self.get_g_rhsTuple(u)
        return min(g_rhsTuple) + heuristic + self.km, min(g_rhsTuple)  #return keys = [k1,k2]

    # we do not define update_vertex, as it can safely inherit from the superclass
    # note that directionality is inverted, but since our mobility graph is undirected, this is okay

    def computeShortestPath(self):
        g_Sstart, rhs_Sstart = self.get_g_rhsTuple(self.start)
        while (self.U.size() > 0 and self.keyComparision(self.U.topKey(), self.calculateKey(self.start))) or g_Sstart != rhs_Sstart:
            k_old = self.U.topKey()
            u = self.U.pop()
            self.popCount += 1
            g_u, rhs_u = self.get_g_rhsTuple(u)
            if self.keyComparision(k_old, self.calculateKey(u)):
                self.U.insert(u, self.calculateKey(u))
            elif g_u > rhs_u:
                # locally overconsistent
                self.set_g_rhsTuple(u, (rhs_u, BENCHMARK))  # g(s) = rhs(s)
            else:
                self.set_g_rhsTuple(u, (float("inf"), BENCHMARK))  # g(s) = infinity
                self.updateVertex(u)  # update the vertex itself
            for s in self._get_neighbors(u):
                self.updateVertex(s)  # update the successor vertices, in either case
            g_Sstart, rhs_Sstart = self.get_g_rhsTuple(self.start)  # update for next iteration ??
            
        self.hasPath = True

    # ########  external (pacman) helper functions  ########
    def nodeUpdate(self, u):
        x, y = u

        startNeighbors = self.getNeighbors(self.start)
        if u not in startNeighbors:
            raise ValueError("A wall cannot be discovered at a non-adjacent location; this breaks D* Lite.")
        self.changedEdges.append(u)

        self.hitwall[x][y] = True
        for neighbor in self.getNeighbors(u):
            self.changedEdges.append(neighbor)  # add all wall-adjacent edges to the queue to be update_vertex'd

        # process edge updates
        self.km += self.h(self.last, self.start)
        self.last = self.start
        for changedEdge in self.changedEdges:
            self.updateVertex(changedEdge, self.goal)
        self.changedEdges = list()  # we've updated all the edges
        self.computeShortestPath()  # find the new shortest path

    def findNewStart(self):
        if self.start == self.goal:
            return self.start  # we're already at the goal; no need to move

        g_Sstart, rhs_Sstart = self.get_g_rhsTuple(self.start)
        if g_Sstart == float("inf"):
            return self.start  # no path exists; no need to move

        # move according to our best guess
        argmin = (None, float("inf"))
        for neighbor in self.getNeighbors(self.start):
            weight = COST + self.get_g_rhsTuple(neighbor)[0]
            if weight < argmin[1]:
                argmin = (neighbor, weight)
        self.path.append(self.start)
        self.start = argmin[0]

        return self.start

    def extract_path(self, placeholder=None):
        return super(DStarLite, self).extract_path(backward=False)  # override gradient direction

    def getRoute(self):
        res = list(self.path)
        res.append(self.start)
        return res
