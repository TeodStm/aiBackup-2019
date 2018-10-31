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
    "***** auta einai ta palia ***"
    node_list = set()# xrhsimopoieitai gia thn euresh tou monopatiou sth sunarthsh solution
    path = list()
    fringe = util.Stack() #DFS uses stack for fringe 
    fringeFlg = False
    explFlg = False
    
    if problem.isGoalState(problem.getStartState()):
        return[]
    fringe.push([problem.getStartState(),None,0]) #[node, action, cost]
    node_list.add((problem.getStartState(),problem.getStartState(),None)) #[node,parent,action]
    
    while True:    
        if fringe.isEmpty(): return []
        testNode = fringe.pop()
        successors = problem.getSuccessors(testNode[0])
        
        for i in successors:
            fringeFlg = False
            explFlg = False
            for f in fringe.list: #check if node is already in fringe
                if i[0] == f[0]:
                    fringeFlg = True
                    break
                
            for e in node_list: #check if node is already in explored nodes
                if i[0] == e[0]:
                    explFlg = True
                    break
            if explFlg == False and fringeFlg == False:
                if problem.isGoalState(i[0]): #if a goal was found , call solution to construct the path
                    state = i[0]
                    parent = testNode[0]
                    action = i[1]
                    node_list.add((state,parent,action))
                    path = solution(node_list, i[0], problem.getStartState())  #construct the path
                    return path
                fringe.push([i[0],i[1],i[2]]) 
                state = i[0]
                parent = testNode[0]
                action = i[1]
                node_list.add((state,parent,action))    
    util.raiseNotDefined()

def solution(expl_nodes, node, startNode): #h sunarthsh auth einai gia thn euresh tou monopatiou mexri to stoxo
    path = list()
    target = node
    while target != startNode:
        for i in expl_nodes:
            if i[0] == target:
                path.append(i[2])
                target = i[1]
                break
    path.reverse()
    return path
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    explored_nodes = set()
    node_list = list() # xrhsimopoieitai gia thn euresh tou monopatiou sth sunarthsh solution
    path = list()
    fringe = util.Queue() #BFS uses queue for fringe
    
    if problem.isGoalState(problem.getStartState()):
        return[]
    fringe.push([problem.getStartState(),None,0]) #[node, action, cost ]
    node_list.append((problem.getStartState(),problem.getStartState(),None)) #[node,parent,action]
    while True:   
        if fringe.isEmpty(): return []
        testNode = fringe.pop()
        successors = problem.getSuccessors(testNode[0])
        for i in successors:
            fringeFlg = False
            explFlg = False
            
            for f in fringe.list: #check if node is already in fringe
                if i[0] == f[0]:
                    fringeFlg = True
                    break

            for e in node_list: #check if node is already in explored nodes
                if i[0] == e[0]:
                    explFlg = True
                    break
                
            if fringeFlg == False and explFlg == False:
                if problem.isGoalState(i[0]): #if a goal was found , call solution to construct the path
                    state = i[0]
                    parent = testNode[0]
                    action = i[1]
                    node_list.append((state,parent,action))
                    path = solution(node_list, i[0], problem.getStartState())
                    return path
                fringe.push([i[0],i[1],i[2]])
                state = i[0]
                parent = testNode[0]
                action = i[1]
                node_list.append((state,parent,action))
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    node_list = set() # xrhsimopoieitai gia thn euresh tou monopatiou sth sunarthsh solution
    path = list()
    fringe = util.PriorityQueue()
    fringe.push([problem.getStartState(),None,0, problem.getStartState()],0) #[node,action,cost,parent]
    explFlg = False
    fringeFlg = False
  
    node_list.add((problem.getStartState(),problem.getStartState(),None))
    
    while True:    
        if fringe.isEmpty(): return []
        testNode = fringe.pop()
        if problem.isGoalState( testNode[0] ): #if a goal was found , call solution to construct the path
            state = testNode[0] #node
            parent = testNode[3] #parent node
            action = testNode[1] #action from parent to child
            node_list.add( (state,parent,action) )
            path = solution(node_list, state, problem.getStartState())
            return path
                
        node_list.add((testNode[0],testNode[3],testNode[1])) #[node,parent,action]
        
        successors = problem.getSuccessors(testNode[0])
        for i in successors:
            
            for f in fringe.heap: #check if node is already in fringe
                if f[0] == i[0]:
                    fringeFlg = True
                    break
            for e in node_list: #check if node is already in explored nodes
                if e[0] == i[0]:
                    explFlg = True
                    break
                
            if explFlg == False and fringeFlg == False:
                state = i[0]
                parent = testNode[0]
                action = i[1]
                cost = i[2]
                fringe.push([state,action,cost,parent],testNode[2]+cost) #eisagwgh sto sunoro
            elif fringeFlg == True:
                state = i[0]
                parent = testNode[0]
                action = i[1]
                cost = i[2]
                item = [state, action, cost, parent] 
                fringe.update(i,testNode[2]+cost) #update sto sunoro(vrethike mikrotero priority)
                
            fringeFlg = False
            explFlg = False
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    node_list = set() # xrhsimopoieitai gia thn euresh tou monopatiou sth sunarthsh solution
    path = list()
    fringe = util.PriorityQueue()
    
    h = heuristic(problem.getStartState(), problem)
    
    if problem.isGoalState(problem.getStartState()):
        return[]
    fringe.push([problem.getStartState(),None,0],0) #( [node, action, cost], f )
  
    node_list.add((problem.getStartState(),problem.getStartState(),None)) #[node, action, cost ]
    
    while True:    
        if fringe.isEmpty():
            return []
        testNode = fringe.pop()
        successors = problem.getSuccessors(testNode[0])
        for i in successors:
            inFringeFlg = False
            explFlg = False
            
            for f in fringe.heap: #check if node is already in fringe
                if i[0] == f[0]:
                    inFringeFlg = True
                    break
            for e in node_list: #check if node is already in explored nodes
                if i[0] == e[0]:
                    explFlg = True
                    break
            if explFlg == False and inFringeFlg == False:
                if problem.isGoalState(i[0]): #if a goal was found , call solution to construct the path
                    state = i[0]
                    parent = testNode[0]
                    action = i[1]
                    node_list.add((state,parent,action))
                    path = solution(node_list, i[0], problem.getStartState()) #construct the path
                    return path
                h = heuristic(i[0], problem)
                fringe.push( [i[0], i[1],i[2]], testNode[2]+i[2]+h )
                state = i[0]
                parent = testNode[0]
                action = i[1]
                node_list.add((state,parent,action))
            elif inFringeFlg == True:
                fringe.update(i, testNode[2])
                
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
