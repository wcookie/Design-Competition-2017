#include <iostream>
#include <math.h>
#include <stdio.h>
#include <ctime>
#include <queue>          // std::queue
#include <list>

using namespace std;

#define WALL_COST_WEIGHT 1 //Cost of being next to a Wall (Corners = 2 X WALL_COST_WEIGHT)
#define EYESIGHT_WEIGHT 1 //Cost of being potentially seen by Enemy
#define ENEMY_PROXIMITY_WEIGHT 20 //COST put on being next to ennemy - decreases w/ Distance
#define ENEMY_DISTANCE 6 //Distance after which enemy proximity cost is not felt
#define DISTANCE_WEIGHT 1 //Cost of being further away from our current position (increases by DISTANCE_WEIGHT for each distance unit)
#define OBSTACLE 1000 //Cost of obstacle - keep really high to signify - don't drive there

#define XROW 9 //Number of Xrows
#define YROW 9 //Number of Yrows

#define SENSORXPOS 4 //Position of sensor (full board) if half board val: 1
#define SENSORYPOS 4 //Position of sensor (full board) if half board val: 4

//point struct for x and y coord, declare currentLoc and currentEnemyLoc
struct point {
  int xpos;
  int ypos;
} currentLoc, currentEnemyLoc;

// An Data Structure for queue used in BFS
struct queueNode
{
    point pt;  // The cordinates of a cell
    int dist;  // cell's distance of from the source
    queueNode* prev;
};


//returns true if point has both xpos and ypos odd
bool isOdd(point coord){
  if (coord.xpos%2 == 1 && coord.ypos%2 == 1){
    return true;
  }
  else return false;
}

//Returns a random starting point for both currentLoc and currentEnemyLoc
void genInitialPoint(){
  bool genPos = false;
  while (!genPos){
    currentLoc.xpos = rand()%(XROW);
    currentLoc.ypos = rand()%(YROW);
    if (!isOdd(currentLoc)){
      genPos = true;
    }
  }
  genPos = false;
  while (!genPos){
    currentEnemyLoc.xpos = rand()%(XROW);
    currentEnemyLoc.ypos = rand()%(YROW);
    if (!isOdd(currentEnemyLoc) && (currentEnemyLoc.xpos != currentLoc.xpos && currentEnemyLoc.ypos != currentLoc.ypos)){ //dont want to initialize robots in same place
      genPos = true;
    }
  }
}


//Prints Cost Matrix in nice format with obstacles, enemy position and our position shown
void printCostMatrix(int** costMatrix){
  for (int h = 0; h < YROW; h++)
  {
    for (int w = 0; w < XROW; w++)
    {
      if (w == currentLoc.xpos && h == currentLoc.ypos)
        printf("X, "); //X for our robot position
      else if (w == currentEnemyLoc.xpos && h == currentEnemyLoc.ypos)
        printf("E, "); //E for enemy
      else if (w%2 == 1 && h%2 == 1)
        printf("C, "); //C for cylinder/cone
      else printf("%i, ", costMatrix[h][w]);
    }
    printf("\n");
  }
}


// check whether given cell (row, col) is a valid - bfs helper
bool isValid(int row, int col)
{
    // return true if row number and column number
    // is in range and no obstacle
    return (row >= 0) && (row < YROW) &&
           (col >= 0) && (col < XROW) &&
           !(row%2 == 1 && col%2 == 1);
}


///Prints list of nodes in reverse order
void print_list(queueNode *traverse)
{
    if (traverse == 0) return;
    print_list(traverse->prev);
    printf("Node x: %d and Node y: %d\n", traverse->pt.xpos, traverse->pt.ypos);
}

//BFS currently returns min cost of path from src to dest
list<point> BFS(int** cost, point src, point dest)
{
    printf("\nInside Finding Min Cost Path\n");
    printf("Current x: %d and Current y: %d\n", src.xpos, src.ypos);
    printf("Target x: %d and Target y: %d\n", dest.xpos, dest.ypos);

    int visited[YROW][XROW];
    memset(visited, false, sizeof visited);

    // Mark the source cell as visited
    visited[src.ypos][src.xpos] = cost[src.ypos][src.xpos];

    // Create a queue for BFS
    queue<queueNode*> q;

    // distance of source cell is 0
    queueNode * s = new queueNode;
    s->pt = src;
    s->dist = 0;
    s->prev = 0;

    // queueNode s = {src, 0, 0};
    q.push(s);  // Enqueue source cell


    queueNode * curr;
    // These arrays are used to get row and column
    // numbers of 4 neighbours of a given cell
    int rowNum[] = {-1, 0, 0, 1};
    int colNum[] = {0, -1, 1, 0};

    // Do a BFS starting from source cell
    while (!q.empty())
    {
        queueNode * curr = q.front();
        point pt = curr->pt;

        // If we have reached the destination cell,
        // we are done
        if (pt.xpos == dest.xpos && pt.ypos == dest.ypos){

            int distance = curr->dist;
            printf("Printing List\n");
            list<point> path;
            while (curr != 0){
              printf("Node x: %d and Node y: %d\n", curr->pt.xpos, curr->pt.ypos);
              path.push_front(curr->pt);
              curr = curr->prev;
            }
            printf("Total distance of Path: %d\n", distance);
            // print_list(curr);
            return path;
          }

        // Otherwise dequeue the front cell in the queue
        // and enqueue its adjacent cells
        q.pop();

        for (int i = 0; i < 4; i++)
        {
            int row = pt.ypos + rowNum[i];
            int col = pt.xpos + colNum[i];

            // if adjacent cell is valid, has path and
            // not visited yet, enqueue it.
            if (isValid(row, col) && !visited[row][col])
            {
                printf("Node x: %d and Node y: %d\n", row, col);
                // mark cell as visited and enqueue it
                visited[row][col] = curr->dist + cost[row][col];

                for (int h = 0; h < YROW; h++)
                {
                  for (int w = 0; w < XROW; w++)
                  {
                    printf("%d, ", visited[h][w]);
                  }
                  printf("\n");
                }

                point adjpoint = {col, row};

                queueNode *Adjcell = new queueNode;
                Adjcell->pt = adjpoint;
                Adjcell->dist = curr->dist + cost[row][col];
                Adjcell->prev = curr;

                q.push(Adjcell);
            }
        }
    }
     //return -1 if destination cannot be reached
     list<point> path;
     path.push_front(src);
     return path;
}


/// This is where we calculate the cost Matrix, based on factors:
// Obstacles
//  -- Walls
//  -- Line of Sight of enemy
//  -- Distance to Enemy
//  -- Distance to Us
// Potential:
//  -- Easyness to get there? Direction?
//  -- Sensor Reach?
list<point> calcCostMatrix(){

  int** costMatrix = 0;
  costMatrix = new int*[YROW];

  int minCost = 1000;
  float minDistCost = 1000.0;
  point target;
  target.xpos = currentLoc.xpos;
  target.ypos = currentLoc.ypos;

  for (int y = 0; y < YROW; y++)
  {
    costMatrix[y] = new int[XROW];
    for (int x = 0; x < XROW; x++)
    {
      //initial Value
      costMatrix[y][x]= 0;

      //obstacles - weight == 1000 - really can't go through them - pathing algo goes around anyways (so doesnt matter can remove)
      if (x%2 == 1 && y%2 == 1)
        costMatrix[y][x] += OBSTACLE;
      else {
        // Add cost if x=0 or x=8
        if (x==0 || x==8)
          costMatrix[y][x] += WALL_COST_WEIGHT;

        //Add cost if y=0 or y=8
        if (y==0 || y==8)
          costMatrix[y][x] += WALL_COST_WEIGHT;

        //line of sight x coord
        if (currentEnemyLoc.xpos == x && x%2 == 0) //if even then not hidden by obstacles
          costMatrix[y][x] += EYESIGHT_WEIGHT;

        // line of sight y coord
        if (currentEnemyLoc.ypos == y && y%2 == 0) //if even then not hidden by obstacles
          costMatrix[y][x] += EYESIGHT_WEIGHT;

        //line of sight diagonal
        if (pow((float)(y-currentEnemyLoc.ypos), 2.0) == pow((float)(x-currentEnemyLoc.xpos), 2.0)
          && !(currentEnemyLoc.xpos%2 == 0 && currentEnemyLoc.ypos%2 == 0)){ //if both enemy coord even then diag l.o.s. is blocked
          costMatrix[y][x] += EYESIGHT_WEIGHT;
        }

        //Increase cost if closer to enemy
        int distToEnemy = abs(y-currentEnemyLoc.ypos)+abs(x-currentEnemyLoc.xpos);
        if (distToEnemy < ENEMY_DISTANCE )
          costMatrix[y][x] += (int)(ENEMY_PROXIMITY_WEIGHT/(distToEnemy+1));

        //Incease cost if farther from us
        int distToUs = abs(y-currentLoc.ypos)+abs(x-currentLoc.xpos);
        costMatrix[y][x] += DISTANCE_WEIGHT*(distToUs+1);

        //Update best min cost and target coord, if current cost is less
        if (costMatrix[y][x] < minCost){
          target.xpos = x;
          target.ypos = y;
          minCost = costMatrix[y][x];
          minDistCost = distToUs/(distToEnemy+1);
        }

        //Update best min cost and target coord if smaller distToUs/(distToEnemy+1)
        if (costMatrix[y][x] == minCost && distToUs/(distToEnemy+1) < minDistCost){
          target.xpos = x;
          target.ypos = y;
          minCost = costMatrix[y][x];
          minDistCost = distToUs/(distToEnemy+1);
        }
      }

    }
  }

  printCostMatrix(costMatrix);

  list<point> path = BFS(costMatrix, currentLoc, target);

  // printf("Path contains: \n");
  //
  // for (auto v : path)
  //   printf("POINT: xpos: %d ; ypos: %d\n", v.xpos, v.ypos );

  // important: clean up memory
  printf("\n");
  printf("Cleaning up cost Array memory...\n");
  for (int h = 0; h < YROW; h++)
  {
    delete [] costMatrix[h];
  }
  delete [] costMatrix;
  costMatrix = 0;

  printf("Returning Best Path.\n");
  return path;
}



int main(int argc, char const *argv[]) {
  //set rand seed
  srand(time(NULL));

  printf("Sup, world!\n");
  genInitialPoint();
  printf("currentLoc xpos: %d ; currentLoc ypos: %d\n", currentLoc.xpos, currentLoc.ypos);
  printf("currentEnemyLocation xpos: %d ; currentEnemyLocation ypos: %d\n", currentEnemyLoc.xpos, currentEnemyLoc.ypos);

  calcCostMatrix(); //Use calcCostMatrix to get Path that should be taken (list of points)



  return 0;
}
