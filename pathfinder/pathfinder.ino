#include <StackArray.h>
#include <QueueArray.h>

#define WALL_COST_WEIGHT 2 //Cost of being next to a Wall (Corners = 2 X WALL_COST_WEIGHT)
#define EYESIGHT_WEIGHT 2 //Cost of being potentially seen by Enemy
#define ENEMY_PROXIMITY_WEIGHT 100 //COST put on being next to ennemy - decreases w/ Distance
#define ENEMY_DISTANCE 6 //Distance after which enemy proximity cost is not felt
#define DISTANCE_WEIGHT 1 //Cost of being further away from our current position (increases by DISTANCE_WEIGHT for each distance unit)
#define OBSTACLE 1000 //Cost of obstacle - keep really high to signify - don't drive there

#define XROW 9 //Number of Xrows
#define YROW 9 //Number of Yrows

#define SENSORXPOS 4 //Position of sensor (full board) if half board val: 1
#define SENSORYPOS 4 //Position of sensor (full board) if half board val: 4

char buffer[200]; 

//point struct for x and y coord, declare currentLoc and currentEnemyLoc
struct point {
  int xpos;
  int ypos;
} currentLoc, currentEnemyLoc;

// An Data Structure for queue used in BFS
struct queueNode
{
    point pt;  // The coordinates of a cell
    int dist;  // cell's distance of from the source
    queueNode* prev;
};
 
typedef struct {
    int priority;
    point* vertex;
} node_t;
 
typedef struct {
    node_t *nodes;
    int len;
    int size;
} heap_t;
 
void pushHeap (heap_t *h, int priority, point* vertex) {
    if (h->len + 1 >= h->size) {
        h->size = h->size ? h->size * 2 : 4;
        h->nodes = (node_t *)realloc(h->nodes, h->size * sizeof (node_t));
    }
    int i = h->len + 1;
    int j = i / 2;
    while (i > 1 && h->nodes[j].priority > priority) {
        h->nodes[i] = h->nodes[j];
        i = j;
        j = j / 2;
    }
    h->nodes[i].priority = priority;
    h->nodes[i].vertex = vertex;
    h->len++;
}

void decrease_priority(heap_t *h, int priority, point* vertex){
   int found = -1;
   for (int i = 1; i < h->len; i++) {
        if (h->nodes[i].vertex->xpos == vertex->xpos && h->nodes[i].vertex->ypos == vertex->ypos)
          found = i;
    }
    if (found != -1){
//      Serial.print("Found\n");
      for (int j = found+1; j < h->len; j++) {
              h->nodes[j - 1] = h->nodes[j];
      }
      h->len--;
    }
    pushHeap(h, priority, vertex);
}

 
point* popHeap (heap_t *h) {
    int i, j, k;
    if (!h->len) {
        return NULL;
    }
    point* vertex = h->nodes[1].vertex;
    h->nodes[1] = h->nodes[h->len];
    h->len--;
    i = 1;
    while (1) {
        k = i;
        j = 2 * i;
        if (j <= h->len && h->nodes[j].priority < h->nodes[k].priority) {
            k = j;
        }
        if (j + 1 <= h->len && h->nodes[j + 1].priority < h->nodes[k].priority) {
            k = j + 1;
        }
        if (k == i) {
            break;
        }
        h->nodes[i] = h->nodes[k];
        i = k;
    }
    h->nodes[i] = h->nodes[h->len + 1];
    return vertex;
}


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
    currentLoc.xpos = (int)random(XROW);
    currentLoc.ypos = (int)random(YROW);
    
//    currentLoc.xpos = 2;
//    currentLoc.ypos = 7;
    if (!isOdd(currentLoc)){
      genPos = true;
    }
  }
  genPos = false;
  while (!genPos){
    currentEnemyLoc.xpos = (int)random(XROW);
    currentEnemyLoc.ypos = (int)random(YROW);
//    currentEnemyLoc.xpos = 2;
//    currentEnemyLoc.ypos = 5;
    if (!isOdd(currentEnemyLoc) && (currentEnemyLoc.xpos != currentLoc.xpos && currentEnemyLoc.ypos != currentLoc.ypos)){ //dont want to initialize robots in same place
      genPos = true;
    }
  }
}


//Prints Cost Matrix in nice format with obstacles, enemy position and our position shown
void printCostMatrix(int costMatrix[][XROW]){
  for (int h = 0; h < YROW; h++)
  {
    for (int w = 0; w < XROW; w++)
    {
      if (w == currentLoc.xpos && h == currentLoc.ypos)
        Serial.print("X, "); //X for our robot position
      else if (w == currentEnemyLoc.xpos && h == currentEnemyLoc.ypos)
        Serial.print("E, "); //E for enemy
      else if (w%2 == 1 && h%2 == 1)
        Serial.print("C, "); //C for cylinder/cone
      else {
        sprintf(buffer, "%i, ", costMatrix[h][w]);
        Serial.print (buffer);
      }
    }
    Serial.print ("\n");
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


//Djikstras currently returns min cost of path from src
StackArray<point> BFS(int cost[][XROW], point src)
{
    Serial.print ("\nInside Finding Min Cost Path\n");
    sprintf(buffer, "Current x: %d and Current y: %d\n", src.xpos, src.ypos);
    Serial.print (buffer);
//    sprintf(buffer, "Target x: %d and Target y: %d\n", dest.xpos, dest.ypos);
//    Serial.print (buffer);


    Serial.print("Creating Priority Queue\n");
    heap_t* Q = new heap_t();

    int distanceArray[YROW][XROW];

    point* prevArray[YROW][XROW];

    for (int k = 0; k < YROW; k++){
     for (int l = 0; l < XROW; l++){
        if (!(k%2 == 1 && l%2 == 1)){
          if (k == src.ypos && l == src.xpos){
            distanceArray[k][l] = 0;
            prevArray[k][l] = 0;
          }
          else {
            distanceArray[k][l] = 1000;
            prevArray[k][l] = new point();
          }
          point* tempPoint = new point();
          tempPoint->xpos = l;
          tempPoint->ypos = k;
          pushHeap(Q, distanceArray[k][l], tempPoint);
        }
        else distanceArray[k][l] = -1;
      }
    }

    for (int h = 0; h < YROW; h++)
    {
      for (int w = 0; w < XROW; w++)
      {
        Serial.print (distanceArray[h][w], DEC);
        Serial.print("\t");  
      }
      Serial.print ("\n");
    }
    
    int rowNum[] = {-1, 0, 0, 1};
    int colNum[] = {0, -1, 1, 0};

    // Do a BFS starting from source cell
    while (Q->len > 0)
    {
        point* u = popHeap(Q);
//        point pt = curr->pt;
     
        for (int i = 0; i < 4; i++)
        {
            int row = u->ypos + rowNum[i];
            int col = u->xpos + colNum[i];

            // if adjacent cell is valid, has path and
            // not visited yet, enqueue it.
            if (isValid(row, col))
            {
//                memset(buffer, 0, sizeof(buffer));
//                sprintf(buffer, "Neighbor Node x: %d and y: %d\n", row, col);
//                Serial.print (buffer);
//
//                int distToUs = abs(row-src.ypos)+abs(col-src.xpos);
//                visited[row][col] = (int)((curr->dist + cost[row][col])/distToUs) + distToUs;

                int alt = distanceArray[u->ypos][u->xpos] + cost[row][col];
                
                if (alt < distanceArray[row][col]){
                  distanceArray[row][col] = alt;
                  prevArray[row][col] = u;
                  point* v = new point();
                  v->xpos = col;
                  v->ypos = row;
                  decrease_priority(Q, alt, v);
                }
                
            }
        }
    }

    for (int h = 0; h < YROW; h++)
    {
      for (int w = 0; w < XROW; w++)
      {
//        Serial.print (distanceArray[h][w], DEC);
        if (h == src.ypos && w == src.xpos) Serial.print("X");
        else if (!(h%2 == 1 && w%2 == 1)){
          sprintf(buffer, "(%d, %d)", prevArray[h][w]->xpos, prevArray[h][w]->ypos);
          Serial.print (buffer);
        }
        else Serial.print("NA");
        Serial.print("\t");  
      }
      Serial.print ("\n");
    }


      
//      Serial.print ("Printing Path from End to start\n");
//
//      //Target for testing 
//      int tempx = 7;
//      int tempy = 2;
//
//      sprintf(buffer, "Total distance of Path: %d\n", distanceArray[tempy][tempx]);
//      Serial.print (buffer);
//      sprintf(buffer, "Node x: %d and Node y: %d\n",tempx, tempy);
//      Serial.print (buffer);
//      
//      while (prevArray[tempy][tempx] != 0){
//        sprintf(buffer, "Node x: %d and Node y: %d\n", prevArray[tempy][tempx]->xpos, prevArray[tempy][tempx]->ypos);
//        Serial.print (buffer);
////        path.push(curr->pt);
//        tempx = prevArray[tempy][tempx]->xpos;
//        tempy = prevArray[tempy][tempx]->ypos;
//      }
//
//
//     //return -1 if destination cannot be reached
     StackArray<point> path;
     path.push(src);
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
StackArray<point> calcCostMatrix(){


  int costMatrix[YROW][XROW];
  memset(costMatrix, 0, sizeof costMatrix);

  int minCost = 1000;
  int maxEnemyDistance = 0;
  point target;
  target.xpos = currentLoc.xpos;
  target.ypos = currentLoc.ypos;

  for (int y = 0; y < YROW; y++)
  {
//    costMatrix[y] = new int[XROW];
    for (int x = 0; x < XROW; x++)
    {
      //initial Value to 1
      costMatrix[y][x]= 1;

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
        if (abs(y-currentEnemyLoc.ypos) == abs(x-currentEnemyLoc.xpos)
          && !(currentEnemyLoc.xpos%2 == 0 && currentEnemyLoc.ypos%2 == 0)){ //if both enemy coord even then diag l.o.s. is blocked
          costMatrix[y][x] += EYESIGHT_WEIGHT;
        }

        //Increase cost if closer to enemy
        int distToEnemy = abs(y-currentEnemyLoc.ypos)+abs(x-currentEnemyLoc.xpos);
        if (distToEnemy < ENEMY_DISTANCE )
          costMatrix[y][x] += (int)(ENEMY_PROXIMITY_WEIGHT/(distToEnemy+1));

        //Incease cost if farther from us
//        int distToUs = abs(y-currentLoc.ypos)+abs(x-currentLoc.xpos);
        // costMatrix[y][x] += DISTANCE_WEIGHT*(distToUs+1);

        //Update best min cost and target coord, if current cost is less
        if (costMatrix[y][x] < minCost){
          target.xpos = x;
          target.ypos = y;
          minCost = costMatrix[y][x];
          maxEnemyDistance = distToEnemy;
        }

        //Update best min cost and target coord if maximizes distance to enemy 
        if (costMatrix[y][x] == minCost && distToEnemy > maxEnemyDistance){
          target.xpos = x;
          target.ypos = y;
          minCost = costMatrix[y][x];
          maxEnemyDistance = distToEnemy;
        }
      }

    }
  }

  printCostMatrix(costMatrix);

  StackArray<point> path = BFS(costMatrix, currentLoc);

  // printf("Path contains: \n");
  //
  // for (auto v : path)
  //   printf("POINT: xpos: %d ; ypos: %d\n", v.xpos, v.ypos );

  memset(costMatrix, 0, sizeof costMatrix);
  return path;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(0));
  
  genInitialPoint();
  
  currentLoc.xpos = 2;
  currentLoc.ypos = 7;
  currentEnemyLoc.xpos = 2;
  currentEnemyLoc.ypos = 5;
  sprintf(buffer, "currentLoc xpos: %d ; currentLoc ypos: %d\n", currentLoc.xpos, currentLoc.ypos);
  Serial.print (buffer);
  sprintf(buffer, "currentEnemyLocation xpos: %d ; currentEnemyLocation ypos: %d\n", currentEnemyLoc.xpos, currentEnemyLoc.ypos);
  Serial.print (buffer);
  calcCostMatrix(); //Use calcCostMatrix to get Path that should be taken (list of points)


}

void loop() {
  // put your main code here, to run repeatedly:

}

