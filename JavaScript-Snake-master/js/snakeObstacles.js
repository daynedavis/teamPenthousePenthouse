/*
JavaScript Snake
By Patrick Gillespie
http://patorjk.com/games/snake
*/

/**
* @module Snake
* @class SNAKE
*/

var SNAKE = SNAKE || {};

/**
* @method addEventListener
* @param {Object} obj The object to add an event listener to.
* @param {String} event The event to listen for.
* @param {Function} funct The function to execute when the event is triggered.
* @param {Boolean} evtCapturing True to do event capturing, false to do event bubbling.
*/

SNAKE.addEventListener = (function() {
    if (window.addEventListener) {
        return function(obj, event, funct, evtCapturing) {
            obj.addEventListener(event, funct, evtCapturing);
        };
    } else if (window.attachEvent) {
        return function(obj, event, funct) {
            obj.attachEvent("on" + event, funct);
        };
    }
})();

/**
* @method removeEventListener
* @param {Object} obj The object to remove an event listener from.
* @param {String} event The event that was listened for.
* @param {Function} funct The function that was executed when the event is triggered.
* @param {Boolean} evtCapturing True if event capturing was done, false otherwise.
*/

SNAKE.removeEventListener = (function() {
    if (window.removeEventListener) {
        return function(obj, event, funct, evtCapturing) {
            obj.removeEventListener(event, funct, evtCapturing);
        };
    } else if (window.detachEvent) {
        return function(obj, event, funct) {
            obj.detachEvent("on" + event, funct);
        };
    }
})();

/**
* This class manages the snake which will reside inside of a SNAKE.Board object.
* @class Snake
* @constructor
* @namespace SNAKE
* @param {Object} config The configuration object for the class. Contains playingBoard (the SNAKE.Board that this snake resides in), startRow and startCol.
*/
SNAKE.Snake = SNAKE.Snake || (function() {
    
    // -------------------------------------------------------------------------
    // Private static variables and methods
    // -------------------------------------------------------------------------
    
    var instanceNumber = 0;
    var blockPool = [];
    
    var SnakeBlock = function() {
        this.elm = null;
        this.elmStyle = null;
        this.row = -1;
        this.col = -1;
        this.xPos = -1000;
        this.yPos = -1000;
        this.next = null;
        this.prev = null;
    };
    
    // this function is adapted from the example at http://greengeckodesign.com/blog/2007/07/get-highest-z-index-in-javascript.html
    function getNextHighestZIndex(myObj) {
        var highestIndex = 0,
            currentIndex = 0,
            ii;
        for (ii in myObj) {
            if (myObj[ii].elm.currentStyle){  
                currentIndex = parseFloat(myObj[ii].elm.style["z-index"],10);
            }else if(window.getComputedStyle) {
                currentIndex = parseFloat(document.defaultView.getComputedStyle(myObj[ii].elm,null).getPropertyValue("z-index"),10);  
            }
            if(!isNaN(currentIndex) && currentIndex > highestIndex){
                highestIndex = currentIndex;
            }
        }
        return(highestIndex+1);  
    }
    
    // -------------------------------------------------------------------------
    // Contructor + public and private definitions
    // -------------------------------------------------------------------------
    
    /*
        config options:
            playingBoard - the SnakeBoard that this snake belongs too.
            startRow - The row the snake should start on.
            startCol - The column the snake should start on.
    */
    return function(config) {
    
        if (!config||!config.playingBoard) {return;}
    
        // ----- private variables -----

        var me = this,
            playingBoard = config.playingBoard,
            myId = instanceNumber++,
            growthIncr = 3,
            moveQueue = [], // a queue that holds the next moves of the snake
            currentDirection = 1, // 0: up, 1: left, 2: down, 3: right
            columnShift = [0, 1, 0, -1],
            rowShift = [-1, 0, 1, 0],
            xPosShift = [],
            yPosShift = [],
            snakeSpeed = 25,
            isDead = false;
        
        // ----- public variables -----

        me.snakeBody = {};
        me.snakeBody["b0"] = new SnakeBlock(); // create snake head
        me.snakeBody["b0"].row = config.startRow || 1;
        me.snakeBody["b0"].col = config.startCol || 1;
        me.snakeBody["b0"].xPos = me.snakeBody["b0"].row * playingBoard.getBlockWidth();
        me.snakeBody["b0"].yPos = me.snakeBody["b0"].col * playingBoard.getBlockHeight();
        me.snakeBody["b0"].elm = createSnakeElement();
        me.snakeBody["b0"].elmStyle = me.snakeBody["b0"].elm.style;
        playingBoard.getBoardContainer().appendChild( me.snakeBody["b0"].elm );
        me.snakeBody["b0"].elm.style.left = me.snakeBody["b0"].xPos + "px";
        me.snakeBody["b0"].elm.style.top = me.snakeBody["b0"].yPos + "px";
        me.snakeBody["b0"].next = me.snakeBody["b0"];
        me.snakeBody["b0"].prev = me.snakeBody["b0"];
        
        me.snakeLength = 1;
        me.snakeHead = me.snakeBody["b0"];
        me.snakeTail = me.snakeBody["b0"];
        me.snakeHead.elm.className = me.snakeHead.elm.className.replace(/\bsnake-snakebody-dead\b/,'');
        me.snakeHead.elm.className += " snake-snakebody-alive";
        
        // ----- private methods -----
        
        function createSnakeElement() {
            var tempNode = document.createElement("div");
            tempNode.className = "snake-snakebody-block";
            tempNode.style.left = "-1000px";
            tempNode.style.top = "-1000px";
            tempNode.style.width = playingBoard.getBlockWidth() + "px";
            tempNode.style.height = playingBoard.getBlockHeight() + "px";
            return tempNode;
        }
        
        function createBlocks(num) {
            var tempBlock;
            var tempNode = createSnakeElement();

            for (var ii = 1; ii < num; ii++){
                tempBlock = new SnakeBlock();
                tempBlock.elm = tempNode.cloneNode(true);
                tempBlock.elmStyle = tempBlock.elm.style;
                playingBoard.getBoardContainer().appendChild( tempBlock.elm );
                blockPool[blockPool.length] = tempBlock;
            }
            
            tempBlock = new SnakeBlock();
            tempBlock.elm = tempNode;
            playingBoard.getBoardContainer().appendChild( tempBlock.elm );
            blockPool[blockPool.length] = tempBlock;
        }
        
    /*===================================================================================================
     *                       A Star Implementation
     *===================================================================================================
     */
                
	/* This function translates things in a coordinate format to a numerical direction where
	 * 0 is North, 1 is East, 2 is South, and 3 is West 
	 */
	function coorToDir (point, nextPoint) {
			 var snakeLength = me.snakeLength;
	         var lastMove = moveQueue[0] || currentDirection;
	
	        // This means that the snake's next move is to the left
	        if (point[0] - nextPoint[0] == 1 && (lastMove !== 1 || snakeLength === 1 ) ){
	           return 3;
	        }
	        // This means that the snake's next move is to the right
	        else if (point[0] - nextPoint[0] == -1 && (lastMove !== 3 || snakeLength === 1)){
	            return 1;
	        }
	        // This means that the snake's next move is down
	        else if ((point[1] - nextPoint[1]) == -1 && (lastMove !== 0 || snakeLength === 1)){
	            return 2;
	        }
	        // This means that the snake's next move is up
	        else {
	            return 0;
	        }
	    } 
	
	// This is the function that uses A* to find the shortest path
	function snakePath(world, pathStart, pathEnd)
	{
	 	// Any tile higher than 0 is considered and object, and all lower are safe to walk on 
		var maxWalkableTileNum = 0;
	 
		// Keep track of the world dimensions
		var worldWidth = world[0].length;
		var worldHeight = world.length;
		var worldSize =	worldWidth * worldHeight;
		
		// Used in finding openNeighbors or allNeighbors
		var findneighbors = function(){};
	
	
		// Calculates Manhattan Distance	
		function ManhattanDistance(Point, Goal)
		{	
			return Math.abs(Point.x - Goal.x) + Math.abs(Point.y - Goal.y);
		}
	 
		// Returns every open neighbor
		function openNeighbors(x, y)
		{
			var	N = y - 1,
			S = y + 1,
			E = x + 1,
			W = x - 1,
			openN = N > -1 && canWalkHere(x, N),
			openS = S < worldHeight && canWalkHere(x, S),
			openE = E < worldWidth && canWalkHere(E, y),
			openW = W > -1 && canWalkHere(W, y),
			result = [];
			if(openN)
			result.push({x:x, y:N});
			if(openE)
			result.push({x:E, y:y,});
			if(openS)
			result.push({x:x, y:S});
			if(openW)
			result.push({x:W, y:y});
			findneighbors(openN, openS, openE, openW, N, S, E, W, result);
			return result;
		}
		
		/* Returns all neighboring tiles and whether or not they are walkable and how many more
		 * walkable neighbors are in that direction
		 */
		function allNeighbors(x, y)
		{
			var	N = y - 1,
			S = y + 1,
			E = x + 1,
			W = x - 1,
			openN = N > -1 && canWalkHere(x, N),
			openS = S < worldHeight && canWalkHere(x, S),
			openE = E < worldWidth && canWalkHere(E, y),
			openW = W > -1 && canWalkHere(W, y),
			result = [];
			result.push({x:x, y:N, on:openN, z:consecutiveneighbors(x,N,"North")});
			result.push({x:E, y:y, on:openE, z:consecutiveneighbors(E,y,"East")});
			result.push({x:x, y:S, on:openS, z:consecutiveneighbors(x,S,"South")});
			result.push({x:W, y:y, on:openW, z:consecutiveneighbors(W,y,"West")});
			findneighbors(openN, openS, openE, openW, N, S, E, W, result);
			return result;
		}
	
		// Returns the number of consecutive walkable neighbors are in a given direction
		function consecutiveneighbors(x,y,direction) 
		{
			var counter = 0;
	        switch (direction) {
	            case "North":
	                while (canWalkHere(x,y) == true){
	                	counter ++;
	                	y -- ;
	                }
	                break;    
	            case "South":
	                while (canWalkHere(x,y) == true){
	                	counter ++;
	                	y ++ ;
	                }
	                break;    
	            case "East":
	                while (canWalkHere(x,y) == true){
	                	counter ++;
	                	x ++ ;
	                }
	                break;    
	            case "West":
	                while (canWalkHere(x,y) == true){
	                	counter ++;
	                	x -- ;
	                }
	                break;  
	            }	
	        return counter;
	    }
	
		// Checks if there is a chute (a oneblock wide tunnel with no escape) in the given direction
		function checkChute(x,y,direction)
		{
	        switch (direction) {
	            case "North":
	                while (worldHeight > y > -1 && worldWidth > x > -1 && canWalkHere(x-1, y) == false && canWalkHere(x+1, y) == false){
	                	if (canWalkHere(x, y - 1) == false){
		                	return true;
	                	}
	                	else{
	                		y -- ;
	                	}
	                }
	                break;    
	            case "South":
	                while (worldHeight > y > -1 && worldWidth > x > -1 && canWalkHere(x-1, y) == false && canWalkHere(x+1, y) == false){
	                	if (canWalkHere(x, y + 1) == false){
		                	return true;
	                	}
	                	else{
	                		y ++ ;
	                	}
	                }
	
	                break;    
	            case "East":
	                while (worldHeight > y > -1 && worldWidth > x > -1 && canWalkHere(x, y-1) == false && canWalkHere(x,y+1) == false){
	                	if (canWalkHere(x + 1, y) == false){
		                	return true;
	                	}
	                	else{
	                		x ++ ;
	                	}
	                }
	
	                break;    
	            case "West":
	                while (worldHeight - 1 > y > 0 && worldWidth - 1 > x > 0 && canWalkHere(x, y-1) == false && canWalkHere(x, y+1) == false){
	                	if (canWalkHere(x - 1, y) == false){
		                	return true;
	                	}
	                	else{
	                		x -- ;
	                	}
	                }
				}
				return false;
		}
	
		// Function that choses a safe direction to move to intelligently stall the snake when the food is unreachable
		function moveSafe()
		{
			/* Checks cases where only option is left or right and chooses option that isn't a chute and has the 
			 * most consecutive walkable neighbors
			 */
			var ns = allNeighbors(me.snakeHead.col, me.snakeHead.row);
			if ((ns[0].on == false) && (ns[1].on == true && ns[3].on == true)) {
				if (ns[3].z > ns[1].z && !(checkChute(ns[3].x,ns[3].y,"West"))) {
					console.log(checkChute(ns[3].x,ns[3].y,"West"));
					moveQueue.unshift(coorToDir(pathStart, [ns[3].x,ns[3].y]));
				}
				else {
					console.log(checkChute(ns[1].x,ns[1].y,"East"));
					moveQueue.unshift(coorToDir(pathStart, [ns[1].x,ns[1].y]));
				}
			}
			/* Checks cases where only option is up or down and chooses option that isn't a chute and has the 
			 * most consecutive walkable neighbors
			 */
			else if ((ns[0].on == true && ns[2].on == true) && (ns[1].on == false && ns[3].on == false)) {
				if (ns[2].z > ns[0].z && !(checkChute(ns[2].x,ns[2].y,"South"))) {
					console.log(checkChute(ns[3].x,ns[3].y,"South"));
					moveQueue.unshift(coorToDir(pathStart, [ns[2].x,ns[2].y]));
				}
				else {
					console.log(checkChute(ns[0].x,ns[0].y,"North"));
					moveQueue.unshift(coorToDir(pathStart, [ns[0].x,ns[0].y]));
				}
			}
			else {
				// Finds direction that is not a chute and walkable.
				ns = allNeighbors(me.snakeHead.col, me.snakeHead.row);
				console.log("else chute check");
				if (ns[0].on && !(checkChute(ns[0].x,ns[0].y,"North"))){
					console.log("North Safe");
					moveQueue.unshift(coorToDir(pathStart, [ns[0].x,ns[0].y]));
				}
				else if (ns[1].on && !(checkChute(ns[1].x,ns[1].y,"East"))){
					console.log("East Safe");
					moveQueue.unshift(coorToDir(pathStart, [ns[1].x,ns[1].y]));
				}
				else if (ns[2].on && !(checkChute(ns[2].x,ns[2].y,"South"))){
					console.log("South Safe");
					moveQueue.unshift(coorToDir(pathStart, [ns[2].x,ns[2].y]));
				}
				else if (ns[3].on && !(checkChute(ns[3].x,ns[3].y,"West"))){
					console.log("West Safe");
					moveQueue.unshift(coorToDir(pathStart, [ns[3].x,ns[3].y]));
				}
				else{
					// Last case scenario where snake is trapped no matter what. Moves to first open tile.
					ns = openNeighbors(me.snakeHead.col, me.snakeHead.row);
					moveQueue.unshift(coorToDir(pathStart, [ns[0].x,ns[0].y]));
				}
			}		
		}
		
		// returns boolean to tell if tile is walkable
		function canWalkHere(x, y)
		{
			return ((world[x] != null) &&
				(world[y][x] <= maxWalkableTileNum));
		};
		
		// Creates a node object with intialized properties
		function Node(Parent, Point)
		{
			var newNode = {
				// pointer to another Node object
				Parent:Parent,
				// array index of this Node in the world linear array
				value:Point.x + (Point.y * worldWidth),
				// the location coordinates of this Node
				x:Point.x,
				y:Point.y,
				// the distanceFunction cost to get
				// TO this Node from the START
				f:0,
				// the distanceFunction cost to get
				// from this Node to the GOAL
				g:0
			};
	 
			return newNode;
		}
		
	    
	
		// Algorithm that implements A* to find shortest path
		function astarPath()
		{
			// create Nodes from the Start and End x,y coordinates
			var	mypathStart = Node(null, {x:pathStart[0], y:pathStart[1]});
			var mypathEnd = Node(null, {x:pathEnd[0], y:pathEnd[1]});
			// create an array that will contain all world cells
			var AStar = new Array(worldSize);
			// list of currently open Nodes
			var Open = [mypathStart];
			// list of closed Nodes
			var Closed = [];
			// list of the final output array
			var result = [];
			
			// reference to a Node (that is nearby)
			var myneighbors;
			// reference to a Node (that we are considering now)
			var myNode;
			// reference to a Node (that starts a path in question)
			var myPath;
			// temp integer variables used in the calculations
			var length, max, min, i, j;
			// iterate through the open list until none are left
			while(length = Open.length)
			{
				max = worldSize;
				min = -1;
				for(i = 0; i < length; i++)
				{
					if(Open[i].f < max)
					{
						max = Open[i].f;
						min = i;
					}
				}
				// grab the next node and remove it from Open array
				myNode = Open.splice(min, 1)[0];
				
				// is it the destination node?
				if(myNode.value === mypathEnd.value)
				{
					
					myPath = Closed[Closed.push(myNode) - 1];
					do
					{
						result.push([myPath.x, myPath.y]);
					}
					while (myPath = myPath.Parent);
					// clear the working arrays
					AStar = Closed = Open = [];
					// we want to return start to finish
					result.reverse();
					
					//console.log(result[0]);
				}
				else // not the destination
				{
					// find which nearby nodes are walkable
					myneighbors = openNeighbors(myNode.x, myNode.y);
					// test each one that hasn't been tried already
					for(i = 0, j = myneighbors.length; i < j; i++)
					{
						myPath = Node(myNode, myneighbors[i]);
						if (!AStar[myPath.value])
						{
							// estimated cost of this particular route so far
							myPath.g = myNode.g + ManhattanDistance(myneighbors[i], myNode);
							// estimated cost of entire guessed route to the destination
							myPath.f = myPath.g + ManhattanDistance(myneighbors[i], mypathEnd);
							// remember this new path for testing above
							Open.push(myPath);
							// mark this node in the world graph as visited
							AStar[myPath.value] = true;
						}
					}
					// close so you know route has been fully tested
					Closed.push(myNode);
				}
			} 
			// If A* was able to find a shortest path, make the snake follow it, else call movesafe()
			if (result[0] != null){
				moveQueue.unshift(coorToDir(result[0],result[1]));
			}
			else {
				moveSafe();
			}
		}
		return astarPath();
	}
	
	/*===================================================================================================
     *                       End of A Star Implementation
     *===================================================================================================
     */
		      
		        
        me.handleArrowKeys = function(keyNum) {
            if (isDead) {return;}
            
            var snakeLength = me.snakeLength;
            var lastMove = moveQueue[0] || currentDirection;

            switch (keyNum) {
                case 37:
                    if ( lastMove !== 1 || snakeLength === 1 ) {
                    //left
                        moveQueue.unshift(3); //SnakeDirection = 3;
                    }
                    break;    
                case 38:
                    if ( lastMove !== 2 || snakeLength === 1 ) {
                    //up
                        moveQueue.unshift(0);//SnakeDirection = 0;
                    }
                    break;    
                case 39:
                    if ( lastMove !== 3 || snakeLength === 1 ) {
                    //right
                        moveQueue.unshift(1); //SnakeDirection = 1;
                    }
                    break;    
                case 40:
                    if ( lastMove !== 0 || snakeLength === 1 ) {
                    //down
                        moveQueue.unshift(2);//SnakeDirection = 2;
                    }
                    break;  
            }
        };
        
        /**
        * This method is executed for each move of the snake. It determines where the snake will go and what will happen to it. This method needs to run quickly.
        * @method go
        */
        var newHead;
        me.go = function() {
        	
        
            var oldHead = me.snakeHead,
                myDirection = currentDirection,
                grid = playingBoard.grid; // cache grid for quicker lookup
            newHead = me.snakeTail;
            me.snakeTail = newHead.prev;
            me.snakeHead = newHead;
            
            //console.log(newHead.row);

        
            // clear the old board position
            if ( grid[newHead.row] && grid[newHead.row][newHead.col] ) {
                grid[newHead.row][newHead.col] = 0;
            }
        
            if (moveQueue.length){
                myDirection = currentDirection = moveQueue.pop();
            }
        
            newHead.col = oldHead.col + columnShift[myDirection];
            newHead.row = oldHead.row + rowShift[myDirection];
            newHead.xPos = oldHead.xPos + xPosShift[myDirection];
            newHead.yPos = oldHead.yPos + yPosShift[myDirection];
            
            if ( !newHead.elmStyle ) {
                newHead.elmStyle = newHead.elm.style;
            }
            
            newHead.elmStyle.left = newHead.xPos + "px";
            newHead.elmStyle.top = newHead.yPos + "px";

            // check the new spot the snake moved into

            if (grid[newHead.row][newHead.col] === 0) {
                grid[newHead.row][newHead.col] = 1;
                setTimeout(function(){me.go();}, snakeSpeed); 
            } else if (grid[newHead.row][newHead.col] > 0) {
                me.handleDeath();
            } else if (grid[newHead.row][newHead.col] === playingBoard.getGridFoodValue()) {
                grid[newHead.row][newHead.col] = 1;
                me.eatFood();
                setTimeout(function(){me.go();}, snakeSpeed);
            } else if (grid[newHead.row][newHead.col] === playingBoard.getGridObstacleValue()) {
            	me.handleDeath();
            }
            
            
            pathStart = [newHead.col, newHead.row];
            pathEnd = [xFood, yFood];
            snakePath(grid, pathStart, pathEnd);
            
        };
        
        /**
        * This method is called when it is determined that the snake has eaten some food.
        * @method eatFood
        */
        me.eatFood = function() {
            if (blockPool.length <= growthIncr) {
                createBlocks(growthIncr*2);
            }
            var blocks = blockPool.splice(0, growthIncr);
            
            var ii = blocks.length,
                index,
                prevNode = me.snakeTail;
            while (ii--) {
                index = "b" + me.snakeLength++;
                me.snakeBody[index] = blocks[ii];
                me.snakeBody[index].prev = prevNode;
                me.snakeBody[index].elm.className = me.snakeHead.elm.className.replace(/\bsnake-snakebody-dead\b/,'')
                me.snakeBody[index].elm.className += " snake-snakebody-alive";
                prevNode.next = me.snakeBody[index];
                prevNode = me.snakeBody[index];
            }
            me.snakeTail = me.snakeBody[index];
            me.snakeTail.next = me.snakeHead;
            me.snakeHead.prev = me.snakeTail;

            playingBoard.foodEaten();
        };
        
        /**
        * This method handles what happens when the snake dies.
        * @method handleDeath
        */
        me.handleDeath = function() {
            me.snakeHead.elm.style.zIndex = getNextHighestZIndex(me.snakeBody);
            me.snakeHead.elm.className = me.snakeHead.elm.className.replace(/\bsnake-snakebody-alive\b/,'')
            me.snakeHead.elm.className += " snake-snakebody-dead";

            isDead = true;
            playingBoard.handleDeath();
            moveQueue.length = 0;
        };

        /**
        * This method sets a flag that lets the snake be alive again.
        * @method rebirth
        */   
        me.rebirth = function() {
            isDead = false;
        };
        
        /**
        * This method reset the snake so it is ready for a new game.
        * @method reset
        */        
        me.reset = function() {
            if (isDead === false) {return;}
            
            var blocks = [],
                curNode = me.snakeHead.next,
                nextNode;
            while (curNode !== me.snakeHead) {
                nextNode = curNode.next;
                curNode.prev = null;
                curNode.next = null;
                blocks.push(curNode);
                curNode = nextNode;
            }
            me.snakeHead.next = me.snakeHead;
            me.snakeHead.prev = me.snakeHead;
            me.snakeTail = me.snakeHead;
            me.snakeLength = 1;
            
            for (var ii = 0; ii < blocks.length; ii++) {
                blocks[ii].elm.style.left = "-1000px";
                blocks[ii].elm.style.top = "-1000px";
                blocks[ii].elm.className = me.snakeHead.elm.className.replace(/\bsnake-snakebody-dead\b/,'')
                blocks[ii].elm.className += " snake-snakebody-alive";
            }
            
            blockPool.concat(blocks);
            me.snakeHead.elm.className = me.snakeHead.elm.className.replace(/\bsnake-snakebody-dead\b/,'')
            me.snakeHead.elm.className += " snake-snakebody-alive";
            me.snakeHead.row = config.startRow || 1;
            me.snakeHead.col = config.startCol || 1;
            me.snakeHead.xPos = me.snakeHead.row * playingBoard.getBlockWidth();
            me.snakeHead.yPos = me.snakeHead.col * playingBoard.getBlockHeight();
            me.snakeHead.elm.style.left = me.snakeHead.xPos + "px";
            me.snakeHead.elm.style.top = me.snakeHead.yPos + "px";
        };
        
        // ---------------------------------------------------------------------
        // Initialize
        // ---------------------------------------------------------------------
        
        createBlocks(growthIncr*2);
        xPosShift[0] = 0;
        xPosShift[1] = playingBoard.getBlockWidth();
        xPosShift[2] = 0;
        xPosShift[3] = -1 * playingBoard.getBlockWidth();
        
        yPosShift[0] = -1 * playingBoard.getBlockHeight();
        yPosShift[1] = 0;
        yPosShift[2] = playingBoard.getBlockHeight();
        yPosShift[3] = 0;
    };
})();

/**
* This class manages the food which the snake will eat.
* @class Food
* @constructor
* @namespace SNAKE
* @param {Object} config The configuration object for the class. Contains playingBoard (the SNAKE.Board that this food resides in).
*/

var xFood;
var yFood;
SNAKE.Food = SNAKE.Food || (function() {

	//console.log("foodhello")
    
    // -------------------------------------------------------------------------
    // Private static variables and methods
    // -------------------------------------------------------------------------
    
    var instanceNumber = 0;
    
    function getRandomPosition(x, y){
        return Math.floor(Math.random()*(y+1-x)) + x; 
    }
    
    // -------------------------------------------------------------------------
    // Contructor + public and private definitions
    // -------------------------------------------------------------------------
    
    /*
        config options:
            playingBoard - the SnakeBoard that this object belongs too.
    */
    return function(config) {
        
        if (!config||!config.playingBoard) {return;}

        // ----- private variables -----

        var me = this;
        var playingBoard = config.playingBoard;
        var fRow, fColumn;
        var myId = instanceNumber++;
        

        var elmFood = document.createElement("div");
        elmFood.setAttribute("id", "snake-food-"+myId);
        elmFood.className = "snake-food-block";
        elmFood.style.width = playingBoard.getBlockWidth() + "px";
        elmFood.style.height = playingBoard.getBlockHeight() + "px";
        elmFood.style.left = "-1000px";
        elmFood.style.top = "-1000px";
        playingBoard.getBoardContainer().appendChild(elmFood);
        
        // ----- public methods -----
        
        /**
        * @method getFoodElement
        * @return {DOM Element} The div the represents the food.
        */        
        me.getFoodElement = function() {
            return elmFood; 
             
        };


        /**
        * Randomly places the food onto an available location on the playing board.
        * @method randomlyPlaceFood
        */    
        me.randomlyPlaceFood = function() {
        	//console.log("hello3")
            // if there exist some food, clear its presence from the board
            if (playingBoard.grid[fRow] && playingBoard.grid[fRow][fColumn] === playingBoard.getGridFoodValue()){
                playingBoard.grid[fRow][fColumn] = 0; 
            }

            var row = 0, col = 0, numTries = 0;

            var maxRows = playingBoard.grid.length-1;
            var maxCols = playingBoard.grid[0].length-1;
            
            while (playingBoard.grid[row][col] !== 0){
                row = getRandomPosition(1, maxRows);
                col = getRandomPosition(1, maxCols);

                // in some cases there may not be any room to put food anywhere
                // instead of freezing, exit out
                numTries++;
                if (numTries > 20000){
                    row = -1;
                    col = -1;
                    break; 
                } 
            }

            playingBoard.grid[row][col] = playingBoard.getGridFoodValue();
            fRow = row;
            fColumn = col;
            elmFood.style.top = row * playingBoard.getBlockHeight() + "px";
            elmFood.style.left = col * playingBoard.getBlockWidth() + "px";
            xFood = col;
            yFood = row;
            
        };
    };
})();


//generating obstacles

var xObstacle;
var yObstalce;
SNAKE.Obstacle = SNAKE.Obstacle || (function() {

	//console.log("hello")
    
    // -------------------------------------------------------------------------
    // Private static variables and methods
    // -------------------------------------------------------------------------
    
    var instanceNumber = 0;
    
    function getRandomPosition(x, y){
        return Math.floor(Math.random()*(y+1-x)) + x; 
    }
    
    // -------------------------------------------------------------------------
    // Contructor + public and private definitions
    // -------------------------------------------------------------------------
    
    /*
        config options:
            playingBoard - the SnakeBoard that this object belongs too.
    */
    return function(config) {
        
        if (!config||!config.playingBoard) {return;}

        // ----- private variables -----

        var me = this;
        var playingBoard = config.playingBoard;
        var oRow, oColumn;
        var myId = instanceNumber++;
        

        var elmObstacle = document.createElement("div");
        elmObstacle.setAttribute("id", "snake-obstacle-"+myId);
        elmObstacle.className = "snake-obstacle-block";
        elmObstacle.style.width = playingBoard.getBlockWidth() + "px";
        elmObstacle.style.height = playingBoard.getBlockHeight() + "px";
        elmObstacle.style.left = "-1000px";
        elmObstacle.style.top = "-1000px";
        playingBoard.getBoardContainer().appendChild(elmObstacle);
        
        // ----- public methods -----
        
        /**
        * @method getObstacleElement
        * @return {DOM Element} The div the represents the obstacle.
        */        
        me.getObstacleElement = function() {
            return elmObstacle; 
             
        };
        
        /**
        * Randomly places the obstacle onto an available location on the playing board.
        * @method randomlyPlaceObstacle
        */    
        me.randomlyPlaceObstacle = function() {
            // if there exist some food, clear its presence from the board
            if (playingBoard.grid[oRow] && playingBoard.grid[oRow][oColumn] === playingBoard.getGridObstacleValue()){
                playingBoard.grid[oRow][oColumn] = 0; 
            }

            var row = 0, col = 0, numTries = 0;

            var maxRows = playingBoard.grid.length-1;
            var maxCols = playingBoard.grid[0].length-1;
            
            while (playingBoard.grid[row][col] !== 0){
                row = getRandomPosition(1, maxRows);
                col = getRandomPosition(1, maxCols);

                // in some cases there may not be any room to put obstacle anywhere
                // instead of freezing, exit out
                numTries++;
                if (numTries > 20000){
                    row = -1;
                    col = -1;
                    break; 
                } 
          }

            playingBoard.grid[row][col] = playingBoard.getGridObstacleValue();
            oRow = row;
            oColumn = col;
            elmObstacle.style.top = row * playingBoard.getBlockHeight() + "px";
            elmObstacle.style.left = col * playingBoard.getBlockWidth() + "px";
            xObstacle = col;
            yObstacle = row;
            
        };
    };
})();

/**
* This class manages playing board for the game.
* @class Board
* @constructor
* @namespace SNAKE
* @param {Object} config The configuration object for the class. Set fullScreen equal to true if you want the game to take up the full screen, otherwise, set the top, left, width and height parameters.
*/

SNAKE.Board = SNAKE.Board || (function() {

    // -------------------------------------------------------------------------
    // Private static variables and methods
    // -------------------------------------------------------------------------

    var instanceNumber = 0;

    // this function is adapted from the example at http://greengeckodesign.com/blog/2007/07/get-highest-z-index-in-javascript.html
    function getNextHighestZIndex(myObj) {
        var highestIndex = 0,
            currentIndex = 0,
            ii;
        for (ii in myObj) {
            if (myObj[ii].elm.currentStyle){  
                currentIndex = parseFloat(myObj[ii].elm.style["z-index"],10);
            }else if(window.getComputedStyle) {
                currentIndex = parseFloat(document.defaultView.getComputedStyle(myObj[ii].elm,null).getPropertyValue("z-index"),10);  
            }
            if(!isNaN(currentIndex) && currentIndex > highestIndex){
                highestIndex = currentIndex;
            }
        }
        return(highestIndex+1);  
    }

    /*
        This function returns the width of the available screen real estate that we have
    */
    function getClientWidth(){
        var myWidth = 0;
        if( typeof window.innerWidth === "number" ) {
            myWidth = window.innerWidth;//Non-IE
        } else if( document.documentElement && ( document.documentElement.clientWidth || document.documentElement.clientHeight ) ) {
            myWidth = document.documentElement.clientWidth;//IE 6+ in 'standards compliant mode'
        } else if( document.body && ( document.body.clientWidth || document.body.clientHeight ) ) {
            myWidth = document.body.clientWidth;//IE 4 compatible
        } 
        return myWidth;
    }
    /*
        This function returns the height of the available screen real estate that we have
    */
    function getClientHeight(){
        var myHeight = 0;
        if( typeof window.innerHeight === "number" ) {
            myHeight = window.innerHeight;//Non-IE
        } else if( document.documentElement && ( document.documentElement.clientWidth || document.documentElement.clientHeight ) ) {
            myHeight = document.documentElement.clientHeight;//IE 6+ in 'standards compliant mode'
        } else if( document.body && ( document.body.clientWidth || document.body.clientHeight ) ) {
            myHeight = document.body.clientHeight;//IE 4 compatible
        } 
        return myHeight;
    }

    // -------------------------------------------------------------------------
    // Contructor + public and private definitions
    // -------------------------------------------------------------------------
    
    return function(inputConfig) {
    
        // --- private variables ---
        var me = this,
            myId = instanceNumber++,
            config = inputConfig || {},
            MAX_BOARD_COLS = 250,
            MAX_BOARD_ROWS = 250,
            blockWidth = 20,
            blockHeight = 20,
            GRID_FOOD_VALUE = -1, // the value of a spot on the board that represents snake food, MUST BE NEGATIVE
            GRID_OBSTACLE_VALUE = 1,
            myFood,
            mySnake,
            //myObstacle,
            //myObstacle2,
            boardState = 1, // 0: in active; 1: awaiting game start; 2: playing game
            myKeyListener,
            // Board components
            elmContainer, elmPlayingField, elmAboutPanel, elmLengthPanel, elmWelcome, elmTryAgain;
        
        var myObstacle = [];
        // --- public variables ---
        me.grid = [];
        
        // ---------------------------------------------------------------------
        // private functions
        // ---------------------------------------------------------------------
        
        function createBoardElements() {
            elmPlayingField = document.createElement("div");
            elmPlayingField.setAttribute("id", "playingField");
            elmPlayingField.className = "snake-playing-field";
            
            SNAKE.addEventListener(elmPlayingField, "click", function() {
                elmContainer.focus();
            }, false);
            
            elmAboutPanel = document.createElement("div");
            elmAboutPanel.className = "snake-panel-component";
            elmAboutPanel.innerHTML = "<a href='http://patorjk.com/blog/software/' class='snake-link'>more patorjk.com apps</a>";
            
            elmLengthPanel = document.createElement("div");
            elmLengthPanel.className = "snake-panel-component";
            elmLengthPanel.innerHTML = "Length: 1";
            
            elmWelcome = createWelcomeElement();
            elmTryAgain = createTryAgainElement();
            
            SNAKE.addEventListener( elmContainer, "keyup", function(evt) {
                if (!evt) var evt = window.event;
                evt.cancelBubble = true;
                if (evt.stopPropagation) {evt.stopPropagation();}
                if (evt.preventDefault) {evt.preventDefault();}
                return false;
            }, false);
            
            elmContainer.className = "snake-game-container";
            
            elmContainer.appendChild(elmPlayingField);
            elmContainer.appendChild(elmAboutPanel);
            elmContainer.appendChild(elmLengthPanel);
            elmContainer.appendChild(elmWelcome);
            elmContainer.appendChild(elmTryAgain);
            
            mySnake = new SNAKE.Snake({playingBoard:me,startRow:2,startCol:2});
            myFood = new SNAKE.Food({playingBoard: me});
	        
	        for(var i = 0; i<20; i++){
	        	myObstacle[i] = new SNAKE.Obstacle({playingBoard: me});
	        }

	        //myObstacle = new SNAKE.Obstacle({playingBoard: me});
 	        //myObstacle2 = new SNAKE.Obstacle({playingBoard: me});   		

            elmWelcome.style.zIndex = 1000;
        }
        function maxBoardWidth() {
            return MAX_BOARD_COLS * me.getBlockWidth();   
        }
        function maxBoardHeight() {
            return MAX_BOARD_ROWS * me.getBlockHeight();
        }
        
        function createWelcomeElement() {
            var tmpElm = document.createElement("div");
            tmpElm.id = "sbWelcome" + myId;
            tmpElm.className = "snake-welcome-dialog";
            
            var welcomeTxt = document.createElement("div");
            var fullScreenText = "";

            welcomeTxt.innerHTML = "JavaScript Snake<p></p>Use an <strong>arrow key</strong> on your keyboard to start the game. Refresh the screen to restart the game. " + fullScreenText + "<p></p>";
            var welcomeStart = document.createElement("button");
            welcomeStart.appendChild( document.createTextNode("Play Game"));
            
            var loadGame = function() {
                SNAKE.removeEventListener(window, "keyup", kbShortcut, false);
                tmpElm.style.display = "none";
                me.setBoardState(1);
                me.getBoardContainer().focus();
            };
            
            var kbShortcut = function(evt) {
                if (!evt) var evt = window.event;
                var keyNum = (evt.which) ? evt.which : evt.keyCode;
                if (keyNum === 32 || keyNum === 13) {
                    loadGame();
                }
            };
            SNAKE.addEventListener(window, "keyup", kbShortcut, false);
            SNAKE.addEventListener(welcomeStart, "click", loadGame, false);
            
            tmpElm.appendChild(welcomeTxt);
            tmpElm.appendChild(welcomeStart);
            return tmpElm;
        }
        
        function createTryAgainElement() {
            var tmpElm = document.createElement("div");
            tmpElm.id = "sbTryAgain" + myId;
            tmpElm.className = "snake-try-again-dialog";
            
            var tryAgainTxt = document.createElement("div");
            tryAgainTxt.innerHTML = "JavaScript Snake<p></p>You died :(. Refresh the page to play again. <p></p>";
            var tryAgainStart = document.createElement("button");
            tryAgainStart.appendChild( document.createTextNode("Play Again?"));
            
            var reloadGame = function() {
                tmpElm.style.display = "none";
                me.resetBoard();
                me.setBoardState(1);
                me.getBoardContainer().focus();
            };
            
            var kbTryAgainShortcut = function(evt) {
                if (boardState !== 0 || tmpElm.style.display !== "block") {return;}
                if (!evt) var evt = window.event;
                var keyNum = (evt.which) ? evt.which : evt.keyCode;
                if (keyNum === 32 || keyNum === 13) {
                    reloadGame();
                }
            };
            SNAKE.addEventListener(window, "keyup", kbTryAgainShortcut, true);
            
            SNAKE.addEventListener(tryAgainStart, "click", reloadGame, false);
            tmpElm.appendChild(tryAgainTxt);
            tmpElm.appendChild(tryAgainStart);
            return tmpElm;
        }
        
        // ---------------------------------------------------------------------
        // public functions
        // ---------------------------------------------------------------------
        
        /**
        * Resets the playing board for a new game.
        * @method resetBoard
        */   
        me.resetBoard = function() {
            SNAKE.removeEventListener(elmContainer, "keydown", myKeyListener, false);
            mySnake.reset();
            elmLengthPanel.innerHTML = "Length: 1";
            me.setupPlayingField();
        };
        /**
        * Gets the current state of the playing board. There are 3 states: 0 - Welcome or Try Again dialog is present. 1 - User has pressed "Start Game" on the Welcome or Try Again dialog but has not pressed an arrow key to move the snake. 2 - The game is in progress and the snake is moving.
        * @method getBoardState
        * @return {Number} The state of the board.
        */  
        me.getBoardState = function() {
            return boardState;
        };
        /**
        * Sets the current state of the playing board. There are 3 states: 0 - Welcome or Try Again dialog is present. 1 - User has pressed "Start Game" on the Welcome or Try Again dialog but has not pressed an arrow key to move the snake. 2 - The game is in progress and the snake is moving.
        * @method setBoardState
        * @param {Number} state The state of the board.
        */  
        me.setBoardState = function(state) {
            boardState = state;
        };
        /**
        * @method getGridFoodValue
        * @return {Number} A number that represents food on a number representation of the playing board.
        */  
        me.getGridFoodValue = function() {
            return GRID_FOOD_VALUE;
        };

        me.getGridObstacleValue = function() {
            return GRID_OBSTACLE_VALUE;
        };

        /**
        * @method getPlayingFieldElement
        * @return {DOM Element} The div representing the playing field (this is where the snake can move).
        */ 
        me.getPlayingFieldElement = function() {
            return elmPlayingField;
        };
        /**
        * @method setBoardContainer
        * @param {DOM Element or String} myContainer Sets the container element for the game.
        */ 
        me.setBoardContainer = function(myContainer) {
            if (typeof myContainer === "string") {
                myContainer = document.getElementById(myContainer);   
            }
            if (myContainer === elmContainer) {return;}
            elmContainer = myContainer;
            elmPlayingField = null;
            
            me.setupPlayingField();
        };
        /**
        * @method getBoardContainer
        * @return {DOM Element}
        */ 
        me.getBoardContainer = function() {
            return elmContainer;
        };
        /**
        * @method getBlockWidth
        * @return {Number}
        */ 
        me.getBlockWidth = function() {
            return blockWidth;  
        };
        /**
        * @method getBlockHeight
        * @return {Number}
        */ 
        me.getBlockHeight = function() {
            return blockHeight;  
        };
        /**
        * Sets up the playing field.
        * @method setupPlayingField
        */ 
        me.setupPlayingField = function () {
            
            if (!elmPlayingField) {createBoardElements();} // create playing field
            
            // calculate width of our game container
            var cWidth, cHeight;
            if (config.fullScreen === true) {
                cTop = 0;
                cLeft = 0;
                cWidth = 750;
                cHeight = 750;
                document.body.style.backgroundColor = "#FC5454";
            } else {
                cTop = config.top;
                cLeft = config.left;
                cWidth = config.width;
                cHeight = config.height;
            }
            
            // define the dimensions of the board and playing field
            var wEdgeSpace = me.getBlockWidth()*2 + (cWidth % me.getBlockWidth());
            var fWidth = Math.min(maxBoardWidth()-wEdgeSpace,cWidth-wEdgeSpace);
            var hEdgeSpace = me.getBlockHeight()*3 + (cHeight % me.getBlockHeight());
            var fHeight = Math.min(maxBoardHeight()-hEdgeSpace,cHeight-hEdgeSpace);
            
            elmContainer.style.left = cLeft + "px";
            elmContainer.style.top = cTop + "px";
            elmContainer.style.width = cWidth + "px";
            elmContainer.style.height = cHeight + "px";
            elmPlayingField.style.left = me.getBlockWidth() + "px";
            elmPlayingField.style.top  = me.getBlockHeight() + "px";
            elmPlayingField.style.width = fWidth + "px";
            elmPlayingField.style.height = fHeight + "px";
            
            // the math for this will need to change depending on font size, padding, etc
            // assuming height of 14 (font size) + 8 (padding)
            var bottomPanelHeight = hEdgeSpace - me.getBlockHeight();
            var pLabelTop = me.getBlockHeight() + fHeight + Math.round((bottomPanelHeight - 30)/2) + "px";
            
            elmAboutPanel.style.top = pLabelTop;
            elmAboutPanel.style.width = "450px";
            elmAboutPanel.style.left = Math.round(cWidth/2) - Math.round(450/2) + "px";
            
            elmLengthPanel.style.top = pLabelTop;
            elmLengthPanel.style.left = cWidth - 120 + "px";
            
            // if width is too narrow, hide the about panel
            if (cWidth < 700) {
                elmAboutPanel.style.display = "none";
            } else {
                elmAboutPanel.style.display = "block";
            }
            
            me.grid = [];
            var numBoardCols = fWidth / me.getBlockWidth() + 2;
            var numBoardRows = fHeight / me.getBlockHeight() + 2;
            
            for (var row = 0; row < numBoardRows; row++) {
                me.grid[row] = [];
                for (var col = 0; col < numBoardCols; col++) {
                    if (col === 0 || row === 0 || col === (numBoardCols-1) || row === (numBoardRows-1)) {
                        me.grid[row][col] = 1; // an edge
                    } else {
                        me.grid[row][col] = 0; // empty space
                    }
                }
            }
            
            myFood.randomlyPlaceFood();
            
            // Generates 20 wall obstacles at start of game
            for (var i = 0; i < 20; i++){
            	myObstacle[i].randomlyPlaceObstacle();
            }

            // setup event listeners
            
            myKeyListener = function(evt) {
                if (!evt) var evt = window.event;
                var keyNum = (evt.which) ? evt.which : evt.keyCode;

                if (me.getBoardState() === 1) {
                    if ( !(keyNum >= 37 && keyNum <= 40) ) {return;} // if not an arrow key, leave
                    
                    // This removes the listener added at the #listenerX line
                    SNAKE.removeEventListener(elmContainer, "keydown", myKeyListener, false);
                    
                    myKeyListener = function(evt) {
                        if (!evt) var evt = window.event;
                        var keyNum = (evt.which) ? evt.which : evt.keyCode;
                        
                        mySnake.handleArrowKeys(keyNum);
                        
                        evt.cancelBubble = true;
                        if (evt.stopPropagation) {evt.stopPropagation();}
                        if (evt.preventDefault) {evt.preventDefault();}
                        return false;
                    };
                    SNAKE.addEventListener( elmContainer, "keydown", myKeyListener, false);
                    
                    mySnake.rebirth();
                    mySnake.handleArrowKeys(keyNum);
                    me.setBoardState(2); // start the game!
                    mySnake.go();
                }
                
                evt.cancelBubble = true;
                if (evt.stopPropagation) {evt.stopPropagation();}
                if (evt.preventDefault) {evt.preventDefault();}
                return false;
            };
            
            // Search for #listenerX to see where this is removed
            SNAKE.addEventListener( elmContainer, "keydown", myKeyListener, false);
        };
        
        /**
        * This method is called when the snake has eaten some food.
        * @method foodEaten
        */ 
        me.foodEaten = function() {
            elmLengthPanel.innerHTML = "Length: " + mySnake.snakeLength;
            myFood.randomlyPlaceFood();
        };
        
        /**
        * This method is called when the snake dies.
        * @method handleDeath
        */ 
        me.handleDeath = function() {
            var index = Math.max(getNextHighestZIndex( mySnake.snakeBody), getNextHighestZIndex( {tmp:{elm:myFood.getFoodElement()}} ));
            elmContainer.removeChild(elmTryAgain);
            elmContainer.appendChild(elmTryAgain);
            elmTryAgain.style.zIndex = index;
            elmTryAgain.style.display = "block";
            me.setBoardState(0);
        };
        
        // ---------------------------------------------------------------------
        // Initialize
        // ---------------------------------------------------------------------

        config.fullScreen = (typeof config.fullScreen === "undefined") ? false : config.fullScreen;        
        config.top = (typeof config.top === "undefined") ? 0 : config.top;
        config.left = (typeof config.left === "undefined") ? 0 : config.left;
        config.width = (typeof config.width === "undefined") ? 400 : config.width;        
        config.height = (typeof config.height === "undefined") ? 400 : config.height;
        
        if (config.fullScreen) {
            SNAKE.addEventListener(window,"resize", function() {
                me.setupPlayingField();
            }, false);
        }
        
        me.setBoardState(0);
        
        if (config.boardContainer) {
            me.setBoardContainer(config.boardContainer);
        }
        
    }; // end return function
})();



