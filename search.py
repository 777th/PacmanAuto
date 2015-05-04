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

def FSNSearch(problem):
    """
    Solve search problem with Functional Systems Network
    """
    import FSNpy as FSN
    from game import Directions

    def outputMap(state, outFS, actions):  # TODO
        """calculates change of the environmental state caused by activities of FSs """
    
        winFS = 0  # probSel(outFS.values())
        for fs in outFS.values():
            if fs.isActive:
                winFS = fs.ID

        action = Directions.NORTH
        if winFS == 2:
            action = Directions.NORTH
        elif winFS == 3:
            action = Directions.EAST
        elif winFS == 4:
            action = Directions.SOUTH
        elif winFS == 5:
            action = Directions.WEST
    
        newState = problem.goTo(state, action)
        if (newState != state):
            actions.append(action)
        
        return newState
    
    start = problem.getStartState()
    goal = problem.goal

    FSNet = FSN.FSNetwork()
    FSNet.initCtrlNet(2, 4, 1)
    FSNet.addActionLinks([[l, FSNet.goalFS.keys()[0], start[l]] for l in range(2)])
    FSNet.addPredictionLinks([[l, FSNet.goalFS.keys()[0], goal[l]] for l in range(2)])

    maxIter = 100  # a period of simulation
    actions = []
    currState = start[:]
    goalsReached = 0
    t = 0
    while goalsReached <= maxIter:
    
        FSNet.update(t, currState)
        currState = outputMap(currState, FSNet.outFS, actions)
        t = t + 1
        if currState == goal:
            goalsReached += 1
            currState = start[:]
            FSNet.resetActivity()
            if goalsReached < maxIter:
                print "Len ", len(actions)
                actions = []
            t = 0
    
    print actions
    return actions

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

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
#    print dir(problem)
#    print "Start:", problem.getStartState()
#    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
#    print "Start's successors:", problem.getSuccessors(problem.getStartState())
#    util.raiseNotDefined()

    def graphSearch(problem):
        closed = util.Counter()
        fringe = util.Stack()
        endnode = (problem.getStartState(),[])
        fringe.push(endnode)
        flag = True
     
        while (flag):
            if (fringe.isEmpty()): 
                flag = False
            else:
                node = fringe.pop()
                if (problem.isGoalState(node[0])):
                    flag  = False
                    endnode = node
                elif (closed[node[0]]==0):
                    closed[node[0]] = 1
                    for child in problem.getSuccessors(node[0]):
                        tmp = list(node[1])
                        tmp.append(child[1])
                        fringe.push((child[0], tmp))
                        
        return endnode[1]
        
    return graphSearch(problem)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    def graphSearch(problem):
        closed = util.Counter()
        fringe = util.Queue()
        endnode = (problem.getStartState(),[])
        fringe.push(endnode)
        flag = True
     
        while (flag):
            if (fringe.isEmpty()): 
                flag = False
#                print "Fail"
            else:
                node = fringe.pop()
#                print ["E: ", node[0]]
                if (problem.isGoalState(node[0])):
                    flag  = False
                    endnode = node
#                    print "Win"
                elif (closed[node[0]]==0):
                    closed[node[0]] = 1
                    for child in problem.getSuccessors(node[0]):
                        tmp = list(node[1])
                        tmp.append(child[1])
                        fringe.push((child[0], tmp))
#                        print ["C: ", child[0]]
#                util.pause()
                        
        return endnode[1]
        
    return graphSearch(problem)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    def graphSearch(problem):
        closed = util.Counter()
        fringe = util.PriorityQueue()
        endnode = (problem.getStartState(),[],0)
        fringe.push(endnode, 0)
        flag = True
     
        while (flag):
            if (fringe.isEmpty()): 
                flag = False
            else:
                node = fringe.pop()
                if (problem.isGoalState(node[0])):
                    flag  = False
                    endnode = node
                elif (closed[node[0]]==0):
                    closed[node[0]] = 1
                    for child in problem.getSuccessors(node[0]):
                        tmp = list(node[1])
                        tmp.append(child[1])
                        fringe.push((child[0], tmp, node[2]+child[2]), node[2]+child[2])
                        
        return endnode[1]
        
    return graphSearch(problem)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    def graphSearch(problem):
        closed = util.Counter()
        fringe = util.PriorityQueue()
        endnode = (problem.getStartState(),[],0)
        fringe.push(endnode, 0)
        flag = True
     
        while (flag):
            if (fringe.isEmpty()): 
                flag = False
            else:
                node = fringe.pop()
                if (problem.isGoalState(node[0])):
                    flag  = False
                    endnode = node
                elif (closed[node[0]]==0):
                    closed[node[0]] = 1
                    for child in problem.getSuccessors(node[0]):
                        tmp = list(node[1])
                        tmp.append(child[1])
                        fringe.push((child[0], tmp, node[2]+child[2]), 
                                node[2]+child[2] + heuristic(child[0],problem))
                        
        return endnode[1]
        
    return graphSearch(problem)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
fsn = FSNSearch
