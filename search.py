# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from collections import deque

# data structure used to store states, parents, actions, and cost
class Node:
  '''
  data structure to be used for all of the search algorithms.
  '''
  def __init__(self, state, parent=None, pathCost=1, prevAction=None):
    self.state = state
    self.parent = parent
    if parent:
      self.totalCost = parent.totalCost + pathCost
    else:
      self.totalCost = 0
    self.prevAction = prevAction

  def getState(self):
    return self.state

  def getParent(self):
    return self.parent

  def getPrevAction(self):
    return self.prevAction

  def getTotalCost(self):
    return self.totalCost


class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

# generic search used by bfs and dfs
def genericSearch(problem, fringeType):
  fringe = fringeType() # stores NODES that we encounter
  firstNode = Node(state=problem.getStartState())
  fringe.push(firstNode)

  explored = [] # stores STATES that have been explored

  while not fringe.isEmpty(): 
    currentNode = fringe.pop()
    # skip node if we have already explored it
    if currentNode.getState() in explored:
      continue
    else:
      currentState = currentNode.getState()
      # if at a goal state, backtrace parent nodes to get complete path taken to goal
      if problem.isGoalState(currentState):
        path = deque()
        while currentNode.getParent():
          path.appendleft(currentNode.getPrevAction())
          currentNode = currentNode.getParent()
        return path
      # if not at goal state, mark current state as explored and add successors to fringe
      else:
        explored.append(currentState)
        for successors in problem.getSuccessors(currentState):
          (state, action, cost) = (successors[0], successors[1], successors[2])
          node = Node(state=state, parent=currentNode, prevAction=action, pathCost=cost)
          fringe.push(node)


def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  return genericSearch(problem, util.Stack)

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  return genericSearch(problem, util.Queue)
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  fringe = util.PriorityQueue() # stores NODES that we encounter
  firstNode = Node(state=problem.getStartState())
  fringe.push(firstNode, 0)

  explored = [] # stores STATES that have been explored

  while not fringe.isEmpty(): 
    currentNode = fringe.pop()
    # skip node if we have already explored it
    if currentNode.getState() in explored:
      continue
    else:
      currentState = currentNode.getState()
      # if at a goal state, backtrace parent nodes to get complete path taken to goal
      if problem.isGoalState(currentState):
        path = deque()
        while currentNode.getParent():
          path.appendleft(currentNode.getPrevAction())
          currentNode = currentNode.getParent()
        return path
      # if not at goal state, mark current state as explored and add successors to fringe
      else:
        explored.append(currentState)
        for successors in problem.getSuccessors(currentState):
          (state, action, cost) = (successors[0], successors[1], successors[2])
          node = Node(state=state, parent=currentNode, prevAction=action, pathCost=cost)
          fringe.push(node, node.getTotalCost())

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  # f(x) = g(x) + h(x). f(x) is stored in the fringe.
  def costPlusHeuristic(node):
    g = node.getTotalCost()
    state = node.getState()
    h = heuristic(state, problem)
    return g + h

  fringe = util.PriorityQueueWithFunction(costPlusHeuristic) # stores NODES that we encounter
  firstNode = Node(state=problem.getStartState())
  fringe.push(firstNode)

  explored = [] # stores STATES that have been explored

  while not fringe.isEmpty(): 
    currentNode = fringe.pop()
    # skip node if we have already explored it
    if currentNode.getState() in explored:
      continue
    else:
      currentState = currentNode.getState()
      # if at a goal state, backtrace parent nodes to get complete path taken to goal
      if problem.isGoalState(currentState):
        path = deque()
        while currentNode.getParent():
          path.appendleft(currentNode.getPrevAction())
          currentNode = currentNode.getParent()
        return path
      # if not at goal state, mark current state as explored and add successors to fringe
      else:
        explored.append(currentState)
        for successors in problem.getSuccessors(currentState):
          (state, action, cost) = (successors[0], successors[1], successors[2])
          node = Node(state=state, parent=currentNode, prevAction=action, pathCost=cost)
          fringe.push(node)
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch