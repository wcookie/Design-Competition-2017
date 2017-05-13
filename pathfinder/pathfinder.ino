#include <MemoryFree.h>
#include <StackArray.h>

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

int distanceArray[YROW][XROW];

//point struct for x and y coord, declare currentLoc and currentEnemyLoc
struct point {
  int xpos;
  int ypos;
};

point* prevArray[YROW][XROW];
point* vertexArray[YROW][XROW];


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

void freeHeap(heap_t *h){
   for (int i = 0; i < h->len; i++){
    point * v = h->nodes[i].vertex;
    delete v;
   }   
   free(h->nodes);
   free(h);
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

bool isEven(point coord){
  if (coord.xpos%2 == 0 && coord.ypos%2 == 0){
    return true;
  }
  else return false;
}


//Prints Cost Matrix in nice format with obstacles, enemy position and our position shown
void printCostMatrix(int **costMatrix, point robotPos, point enemyPos){
  for (int h = 0; h < YROW; h++)
  {
    for (int w = 0; w < XROW; w++)
    {
      if (w == robotPos.xpos && h == robotPos.ypos)
        Serial.print("X, "); //X for our robot position
      else if (w == enemyPos.xpos && h == enemyPos.ypos)
        Serial.print("E, "); //E for enemy
      else if (w%2 == 1 && h%2 == 1)
        Serial.print("C, "); //C for cylinder/cone
      else {
        sprintf(buffer, "%i, ", costMatrix[h][w]);
        Serial.print (buffer);
      }
    }
//    Serial.print("\nfree ram: ");
//    Serial.print(freeMemory());
    Serial.print("\n");
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

StackArray<point> findBestPath(int distanceArray[][XROW], point* prevArray[][XROW], point target){

   for (int h = 0; h < YROW; h++)
    {
      for (int w = 0; w < XROW; w++)
      {
//        if (h == src.ypos && w == src.xpos) Serial.print("X");
        if (!(h%2 == 1 && w%2 == 1)){
          sprintf(buffer, "(%d, %d)", prevArray[h][w]->xpos, prevArray[h][w]->ypos);
          Serial.print (buffer);
        }
        else Serial.print("NA");
        Serial.print("\t");  
      }
      Serial.print ("\n");
    }

   for (int h = 0; h < YROW; h++)
    {
      for (int w = 0; w < XROW; w++)
      {
//        if (h == src.ypos && w == src.xpos) Serial.print("X");
        if (!(h%2 == 1 && w%2 == 1)){
          Serial.print (distanceArray[h][w], DEC);
        }
        else Serial.print("NA");
        Serial.print("\t");  
      }
      Serial.print ("\n");
    }

    Serial.print ("\nPrinting Path from End to start\n");

    //Target for testing 
    int tempx = target.xpos;
    int tempy = target.ypos;
    

    StackArray<point> path;
    
    if (prevArray[tempy][tempx]->xpos == -1){
      point src = {tempx, tempy};
      path.push(src);
    }
    
    sprintf(buffer, "Total distance of Path: %d\n", distanceArray[tempy][tempx]);
    Serial.print (buffer);
    sprintf(buffer, "Target x: %d and y: %d\n",tempx, tempy);
    Serial.print (buffer);

    int tx; 
    while (prevArray[tempy][tempx]->xpos != -1){
      sprintf(buffer, "Node x: %d and Node y: %d\n", prevArray[tempy][tempx]->xpos, prevArray[tempy][tempx]->ypos);
      Serial.print (buffer);
      
      point p = {tempx, tempy};
      path.push(p);
      
      tx = tempx;
      tempx = prevArray[tempy][tempx]->xpos;
      tempy = prevArray[tempy][tx]->ypos;
    }

   return path;
}


//Djikstras currently returns min cost of path from src
StackArray<point> djikstra(int **cost, point src, point target)
{

    Serial.print("\nBEFORE DJIKSTRA: ");
    Serial.print(freeMemory());
    Serial.print("\n");
  
    Serial.print ("\nInside Finding Min Cost Path\n");
    sprintf(buffer, "Current x: %d and Current y: %d\n", src.xpos, src.ypos);
    Serial.print (buffer);
//    sprintf(buffer, "Target x: %d and Target y: %d\n", dest.xpos, dest.ypos);
  //  Serial.print (buffer);

    Serial.print("Creating Priority Queue\n");
    heap_t* Q = new heap_t();

    Serial.print("\nfree ram after Prio Queue: ");
    Serial.print(freeMemory());
    Serial.print("\n");

//    int **distanceArray = (int **) malloc(sizeof(int *) * YROW); 
//    for(int i = 0; i < YROW ; i++){
//       distanceArray[i] = (int*) malloc(sizeof(int)* XROW);     
//    }

//    point **prevArray = (point **) malloc(sizeof(point *) * YROW); 
//    for(int i = 0; i < YROW ; i++){
//       prevArray[i] = (point*) malloc(sizeof(point)* XROW);     
//    }
    Serial.print("\nBEFORE NEW POINT: ");
    Serial.print(freeMemory());
    Serial.print("\n");
    for (int k = 0; k < YROW; k++){
     for (int l = 0; l < XROW; l++){
        if (!(k%2 == 1 && l%2 == 1)){
//          point tempPoint = {l, k};

          if (k == src.ypos && l == src.xpos){
            distanceArray[k][l] = 0;
            prevArray[k][l]->xpos = -1;
            prevArray[k][l]->ypos = -1;
          }
          else {
            distanceArray[k][l] = 1000;
            prevArray[k][l]->xpos = l;
            prevArray[k][l]->ypos = k;
          }
          point* tempPoint = new point();

          tempPoint->xpos = l;
          tempPoint->ypos = k;
          
          pushHeap(Q, distanceArray[k][l], tempPoint);
        }
        else distanceArray[k][l] = -1;
      }
    }
    Serial.print("\nAFTER NEW POINT: ");
    Serial.print(freeMemory());
    Serial.print("\n");


//    Serial.print("\nfree ram after Array declaration: ");
//    Serial.print(freeMemory());
//    Serial.print("\n");
    
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
//              int distToUs = abs(row-src.ypos)+abs(col-src.xpos);
//              visited[row][col] = (int)((curr->dist + cost[row][col])/distToUs) + distToUs;

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
//        delete u;
    }


    Serial.print("\nbefore free Heap: ");
    Serial.print(freeMemory());
    Serial.print("\n");
    freeHeap(Q);
    Serial.print("\nAfter free heap: ");
    Serial.print(freeMemory());
    Serial.print("\n");



    
    StackArray<point> path = findBestPath(distanceArray, prevArray, target);
    
//    for(int i = 0; i < YROW; i++){
//      int* currentIntPtr = distanceArray[i];
//      free(currentIntPtr);
//    } 
//    free(distanceArray);


    
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
StackArray<point> calcCostMatrix(point robotPos, point enemyPos){

  int **costMatrix = (int **) malloc(sizeof(int *) * YROW); 
  for(int i = 0; i < YROW ; i++){
     costMatrix[i] = (int*) malloc(sizeof(int)* XROW);     
  }
  Serial.print("post mallocs \r\n");

  int minCost = 1000;
  int maxEnemyDistance = 0;
  point target;
  target.xpos = robotPos.xpos;
  target.ypos = robotPos.ypos;

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
        if (enemyPos.xpos == x && x%2 == 0) //if even then not hidden by obstacles
          costMatrix[y][x] += EYESIGHT_WEIGHT;

        // line of sight y coord
        if (enemyPos.ypos == y && y%2 == 0) //if even then not hidden by obstacles
          costMatrix[y][x] += EYESIGHT_WEIGHT;

        //line of sight diagonal
        if (abs(y-enemyPos.ypos) == abs(x-enemyPos.xpos)
          && !(enemyPos.xpos%2 == 0 && enemyPos.ypos%2 == 0)){ //if both enemy coord even then diag l.o.s. is blocked
          costMatrix[y][x] += EYESIGHT_WEIGHT;
        }

        //Increase cost if closer to enemy
        int distToEnemy = abs(y-enemyPos.ypos)+abs(x-enemyPos.xpos);
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
  Serial.print("Pre print \r\n");
  printCostMatrix(costMatrix, robotPos, enemyPos);
  Serial.print("PRe djikstra \r\n");
  StackArray<point> path = djikstra(costMatrix, robotPos, target);
  Serial.print("DJIKSTRA DONE \r\n");

  for(int i = 0; i < YROW; i++){
    int* currentIntPtr = costMatrix[i];
    free(currentIntPtr);
  }
  free(costMatrix);
  Serial.print("LEAVING: \r\n");
  Serial.print(freeMemory());
  return path;
}

int getIndex(point src, point target){
  if(isEven(src) && isEven(target)){
    return ((src.xpos/2) + 5 * ((src.ypos/2) + 5 * ((target.xpos/2) + 5 * (target.ypos/2))));
  }
  else {
    Serial.print("\nInvalid points - not Even\n");
    return 0;
  }
}

void preCalculateMatrix(){
  point* PATHMATRIX[625][2];
    
  int index;
//  point enemyLoc, pos;
  for (int i = 0; i < YROW; i = i+2){
    for (int j = 0; j < XROW; j = j+2){
      for (int k = 0; k < YROW; k = k+2){
        for (int l = 0; l < XROW; l = l+2){
          Serial.print("\nCalculating Matrix\n");
          sprintf(buffer, "currentLoc xpos: %d ; currentLoc ypos: %d\n", j, i);
          Serial.print (buffer);
        sprintf(buffer, "currentEnemyLocation xpos: %d ; currentEnemyLocation ypos: %d\n", l, k);
          Serial.print (buffer);
//          Serial.print(freeRam());
          Serial.print("freeMemory()=");
          Serial.println(freeMemory());
//    
          point pos = {j, i};
          point enemyLoc = {l, k};
          index = getIndex(pos, enemyLoc);
          Serial.print("before calcCost \r\n");
          StackArray<point> path = calcCostMatrix(pos, enemyLoc); //Use calcCostMatrix to get Path that should be taken (list of points)
           Serial.print("AFTER: \r\n");
          point top = path.pop();
          sprintf(buffer, "Top xpos: %d ; Top ypos: %d\n", top.xpos, top.ypos);
          Serial.print (buffer);
          PATHMATRIX[index][0] = &top;
    
          ///Has another direction to Go
          if (!path.isEmpty()){
            point secondtop = path.pop();
            sprintf(buffer, "secondtop xpos: %d ; secondtop ypos: %d\n", secondtop.xpos, secondtop.ypos);
            Serial.print (buffer);
            PATHMATRIX[index][1] = &secondtop;        
          }
          else {  //IS already At best direction
            PATHMATRIX[index][1] = &top;
          }
        }
//        memset(buffer, 0, sizeof buffer);

      }
    }
  }
}

void initDistancePrevArrays(){
  for (int k = 0; k < YROW; k++){
     for (int l = 0; l < XROW; l++){
        if (!(k%2 == 1 && l%2 == 1)){
            distanceArray[k][l] = 1000;
            prevArray[k][l] = new point();
            vertexArray[k][l] = new point();          
            vertexArray[k][l]->xpos = l;
            vertexArray[k][l]->ypos = k;
        }
        else {
            distanceArray[k][l] = -1;
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
  
//  calcCostMatrix(currentLoc, currentEnemyLoc); //Use calcCostMatrix to get Path that should be taken (list of points)

    initDistancePrevArrays();
    preCalculateMatrix();

   
//  Serial.print("\nGetting Index: \n");
//  int index = getIndex(currentLoc, currentEnemyLoc);
//  Serial.print(index, DEC);

}

void loop() {
  // put your main code here, to run repeatedly:

}


//    Flat[x + WIDTH * (y + DEPTH * (z + TIME * t ))] = Original[x, y, z, t]


