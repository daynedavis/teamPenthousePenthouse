/*
This is the basic code we have to use astar to find the shortest path. The helper functions find out whether
there are neighbors and then executes the astar algorithm in calculate path using a while loop
*/

//distance function calculates the grid distance between the snake's current head position and the end goal
function distance(pos, goal) {
	abs(pos.x - goal.x) + abs(pos.y - goal.y)
}

function isClear(x, y){
	if (grid[x][y] != NULL){
		true
	} 
	else false
}

//function to check which surrounding cells are clear of obstacles
function neighbors (x, y){

}

//puts everything together
function calculatePath (){
	var AStar = new Array(gridSize);
}