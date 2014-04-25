/* CS51 Final Project
 * A* algorithm 
 * Dylan Peterson
 */

/* takes input function "get_state" */
function gen_search (var, type) {
 	nodes = NodeType();
 	firstNode = Node(state=problem.getStartState());
 	nodes.push(firstNode);

 	explored = [];

 	while not fringe.isEmpty() {
 	  currentNode = fringe.pop();

 	  	if currentNode.getState() in explored {
 	  		continue;
		}	
 	  	else {
 	  		currentState = currentNode.getState();

 	  		if problem.isGoalState(currentState) {
 	  			path = deque ();
 	  			while currentNode.getParent() {
 	  				path.appendleft(currentNode.getPrevAction());
 	  				currentNode = currentNode.getParent();
 	  					return path;
  				}
			}
      		else {
        		explored.append(currentState)
        		for successors in problem.getSuccessors(currentState) {
          			(state, action, cost) = (successors[0], 
          					 successors[1], successors[2]);
          			node = Node(state=state, parent=currentNode,
          					 prevAction=action, pathCost=cost);
          			fringe.push(node);
          		}
      		}
		}
  	}
}

function BreathFirstSearch (var) {
   return genericSearch(problem, util.Queue);
}

function DepthFirstSearch (var) {
	return genericSearch(problem, util.Stack);
}

function uniformCostSearch(problem) {
  fringe = util.PriorityQueue();
  firstNode = Node(state=problem.getStartState());
  fringe.push(firstNode, 0);

  explored = [];

  while not fringe.isEmpty() {
    	currentNode = fringe.pop();
    	if currentNode.getState() in explored {
      		continue;
      	}
    	else {
      		currentState = currentNode.getState();
      		if problem.isGoalState(currentState) {
        		path = deque();
        		while currentNode.getParent() {
          			path.appendleft(currentNode.getPrevAction());
          			currentNode = currentNode.getParent();
        			return path;
        		}
        	}
      		else {
        		explored.append(currentState);
        		for successors in problem.getSuccessors(currentState) {
          			(state, action, cost) = (successors[0], 
          					 successors[1], successors[2]);
          			node = Node(state=state, parent=currentNode, 
          					 prevAction=action, pathCost=cost);
          			fringe.push(node, node.getTotalCost());
          		}
          	}
      	}
  	}
}

function nullHeuristic(state, problem = None) {
  return 0
}

function aStarSearch(problem, heuristic=nullHeuristic) {
  	def costPlusHeuristic(node);
    g = node.getTotalCost();
    state = node.getState();
    h = heuristic(state, problem);
    return g + h;

  	fringe = util.PriorityQueueWithFunction(costPlusHeuristic);
  	firstNode = Node(state=problem.getStartState());
  	fringe.push(firstNode);

  	explored = [];

  	while not fringe.isEmpty(){
    	currentNode = fringe.pop();
    	if currentNode.getState() in explored {
      		continue;
      	}
    	else {
      		currentState = currentNode.getState();
      		if problem.isGoalState(currentState) {
        		path = deque()
        		while currentNode.getParent() {
          			path.appendleft(currentNode.getPrevAction());
          			currentNode = currentNode.getParent();
        			return path;
        		}
    		}
      		else {
        		explored.append(currentState)
        		for successors in problem.getSuccessors(currentState) {
          			(state, action, cost) = (successors[0], 
          					successors[1], successors[2]);
          			node = Node(state=state, parent=currentNode, 
          					prevAction=action, pathCost=cost);
          			fringe.push(node);
          		}
      		}
  		}
    }
}
    