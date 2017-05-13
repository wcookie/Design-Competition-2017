#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <MemoryFree.h>
#include <StackArray.h>


LSM9DS1 imu;


// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


#define PRINT_CALCULATED

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -3.58 // Declination (degrees) in Boulder, CO.

#define XROW 9
#define YROW 9

#define V1PIN 12
#define V2PIN 11 // the signal from the sensor
#define V3PIN 2
#define DEG_PER_US 0.0216 // (180 deg) / (8333 us)
#define LIGHTHOUSEHEIGHT 6.0
#define NEUTRAL_POWER 150
#define EXTREME_NEUTRAL_POWER 150
#define slightkP 2
#define extremekP .8
#define extremekI 0
#define NEW_DISTANCE_WEIGHT 5.0
#define MY_DISTANCE_WEIGHT 1.5
#define NEW_WALL_WEIGHT 1.0



#define WALL_COST_WEIGHT 2 //Cost of being next to a Wall (Corners = 2 X WALL_COST_WEIGHT)
#define EYESIGHT_WEIGHT 2 //Cost of being potentially seen by Enemy
#define ENEMY_PROXIMITY_WEIGHT 100 //COST put on being next to ennemy - decreases w/ Distance
#define ENEMY_DISTANCE 6 //Distance after which enemy proximity cost is not felt
#define DISTANCE_WEIGHT 1 //Cost of being further away from our current position (increases by DISTANCE_WEIGHT for each distance unit)
#define OBSTACLE 1000 //Cost of obstacle - keep really high to signify - don't drive there

#define SENSORXPOS 4 //Position of sensor (full board) if half board val: 1
#define SENSORYPOS 4 //Position of sensor (full board) if half board val: 4


typedef struct {
  unsigned long changeTime[11];
  int prevPulse;
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;

typedef struct {
  double x;
  double y;
}lightPoint;

//point struct for x and y coord, declare currentLoc and currentEnemyLoc
struct point {
  int xpos;
  int ypos;
};


typedef struct {
    int priority;
    point vertex;
} node_t;
 
typedef struct {
    node_t *nodes;
    int len;
    int size;
} heap_t;


char buffer[200]; 
int distanceArray[YROW][XROW];
point prevArray[YROW][XROW];
point vertexArray[YROW][XROW];
point PATHMATRIX[625][2];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////DJIKSTRA CODE///////////////////////////////
void pushHeap (heap_t *h, int priority, point vertex) {
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
    h->nodes[i].vertex.xpos = vertex.xpos;
    h->nodes[i].vertex.ypos = vertex.ypos;
    h->len++;
}

void decrease_priority(heap_t *h, int priority, point vertex){
   int found = -1;
   for (int i = 1; i < h->len; i++) {
        if (h->nodes[i].vertex.xpos == vertex.xpos && h->nodes[i].vertex.ypos == vertex.ypos)
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
//   for (int i = 0; i < h->len; i++){
//    point * v = h->nodes[i].vertex;
//    delete v;
//   }   
   free(h->nodes);
   delete h;
}

point popHeap (heap_t *h) {
    int i, j, k;
    if (!h->len) {
        point emptyPoint = {-2, -2};
        return emptyPoint;
    }
    point vertex = h->nodes[1].vertex;
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

StackArray<point> findBestPath(point target){

   for (int h = 0; h < YROW; h++)
    {
      for (int w = 0; w < XROW; w++)
      {
//        if (h == src.ypos && w == src.xpos) Serial.print("X");
        if (!(h%2 == 1 && w%2 == 1)){
          sprintf(buffer, "(%d, %d)", prevArray[h][w].xpos, prevArray[h][w].ypos);
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
    
    if (prevArray[tempy][tempx].xpos == -1){
      point src = {tempx, tempy};
      path.push(src);
    }
    
    sprintf(buffer, "Total distance of Path: %d\n", distanceArray[tempy][tempx]);
    Serial.print (buffer);
    sprintf(buffer, "Target x: %d and y: %d\n",tempx, tempy);
    Serial.print (buffer);

    int tx; 
    while (prevArray[tempy][tempx].xpos != -1){
      sprintf(buffer, "Node x: %d and Node y: %d\n", prevArray[tempy][tempx].xpos, prevArray[tempy][tempx].ypos);
      Serial.print (buffer);
      
      point p = {tempx, tempy};
      path.push(p);
      
      tx = tempx;
      tempx = prevArray[tempy][tempx].xpos;
      tempy = prevArray[tempy][tx].ypos;
    }

   return path;
}


//Djikstras currently returns min cost of path from src
StackArray<point> djikstra(int **cost, point src, point target)
{
  
    Serial.print ("\nInside Finding Min Cost Path\n");
    sprintf(buffer, "Current x: %d and Current y: %d\n", src.xpos, src.ypos);
    Serial.print (buffer);

    heap_t* Q = new heap_t();

    for (int k = 0; k < YROW; k++){
     for (int l = 0; l < XROW; l++){
        if (!(k%2 == 1 && l%2 == 1)){
          if (k == src.ypos && l == src.xpos){
            distanceArray[k][l] = 0;
            prevArray[k][l].xpos = -1;
            prevArray[k][l].ypos = -1;
          }
          else {
            distanceArray[k][l] = 1000;
            prevArray[k][l].xpos = l;
            prevArray[k][l].ypos = k;
          }
            point tempPoint = {l, k};
          pushHeap(Q, distanceArray[k][l], tempPoint);
        }
        else distanceArray[k][l] = -1;
      }
    }

    
    int rowNum[] = {-1, 0, 0, 1};
    int colNum[] = {0, -1, 1, 0};

    // Do a BFS starting from source cell
    while (Q->len > 0)
    {
        point u = popHeap(Q);
            
        for (int i = 0; i < 4; i++)
        {
            int row = u.ypos + rowNum[i];
            int col = u.xpos + colNum[i];

            // if adjacent cell is valid, has path and
            // not visited yet, enqueue it.
            if (isValid(row, col))
            {
//              int distToUs = abs(row-src.ypos)+abs(col-src.xpos);
//              visited[row][col] = (int)((curr->dist + cost[row][col])/distToUs) + distToUs;

                int alt = distanceArray[u.ypos][u.xpos] + cost[row][col];
                
                if (alt < distanceArray[row][col]){
                  distanceArray[row][col] = alt;
                  prevArray[row][col] = u;
                    point v = {col, row};
                  decrease_priority(Q, alt, v);
                }
                
            }
        }
    }

    freeHeap(Q);
    
    StackArray<point> path = findBestPath(target);
        
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
  printCostMatrix(costMatrix, robotPos, enemyPos);
  
  StackArray<point> path = djikstra(costMatrix, robotPos, target);
  Serial.print("DJIKSTRA DONE \r\n");

  for(int i = 0; i < YROW; i++){
    int* currentIntPtr = costMatrix[i];
    free(currentIntPtr);
  }
  free(costMatrix);
  return path;
}

int getIndex(int ourCurrX, int ourCurrY, int theirCurrX, int theirCurrY){
  if((ourCurrX%2 == 0 && ourCurrY%2 == 0) && (theirCurrX%2 == 0 && theirCurrY%2 == 0)){
    return ((ourCurrX/2) + 5 * ((ourCurrY/2) + 5 * ((theirCurrX/2) + 5 * (theirCurrY/2))));
  }
  else {
    Serial.print("\nInvalid points - not Even\n");
    return 0;
  }
}

void preCalculateMatrix(){
  initDistancePrevArrays();
    
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

          point pos = {j, i};
          point enemyLoc = {l, k};
          index = getIndex(j, i, l, k);
          Serial.print("\nBefore CalcCost \r\n");
          StackArray<point> path = calcCostMatrix(pos, enemyLoc); //Use calcCostMatrix to get Path that should be taken (list of points)
          Serial.print("AFTER: \r\n");
          point top = path.pop();
          sprintf(buffer, "Top xpos: %d ; Top ypos: %d\n", top.xpos, top.ypos);
          Serial.print (buffer);
          PATHMATRIX[index][0] = top;
    
          ///Has another direction to Go
          if (!path.isEmpty()){
            point secondtop = path.pop();
            sprintf(buffer, "secondtop xpos: %d ; secondtop ypos: %d\n", secondtop.xpos, secondtop.ypos);
            Serial.print (buffer);
            PATHMATRIX[index][1] = secondtop;        
          }
          else {  //IS already At best direction
            PATHMATRIX[index][1] = top;
          }
        }
      }
    }
  }
}

void initDistancePrevArrays(){
  for (int k = 0; k < YROW; k++){
     for (int l = 0; l < XROW; l++){
        if (!(k%2 == 1 && l%2 == 1)){
            distanceArray[k][l] = 1000;
            point tempPoint = {l, k};
            prevArray[k][l] = tempPoint;
            vertexArray[k][l] = tempPoint;
        }
        else {
            distanceArray[k][l] = -1;
      }
    }
  }
}

////////////////////////////////////////////////END OF DJIKSTRA CODE ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool calibGyro = true;
volatile viveSensor V1;
volatile viveSensor V2;
volatile viveSensor V3;
unsigned long prevTime = 0;
unsigned long prevTime2 = 0;
unsigned long prevTime3 = 0;
unsigned long lastYawTime = 0;
unsigned long lastEnemyTime = 0;
unsigned long lastFilt1Time = 0;
unsigned long lastFilt2Time = 0;
unsigned long lastFilt3Time = 0;
int state = 0;
double xCombo = 0, yCombo = 0;
double xOld1 = 0, yOld1 = 0, xFilt1 = 0, yFilt1 = 0;
double xOld2 = 0, yOld2 = 0, xFilt2 = 0, yFilt2 = 0;
double xOld3 = 0, yOld3 = 0, xFilt3 = 0, yFilt3 = 0;
double enemyX, enemyY;
lightPoint enemyArr[10];
lightPoint filterPosArr[30];
char filter1PosIndex = 0;
char filter2PosIndex = 0;
char filter3PosIndex = 0;
char enemyIndex = 0;
double desiredYaw = 0 ;
double gyroOffset = 0;
short ourLastX = 2; // STARTING POSITIONS HERE:
short ourLastY = 4;
short theirLastX = 4;
short theirLastY = 4 ;
short ourCurrX;
short ourCurrY;
short theirCurrX;
short theirCurrY;
char msg[100];
char msg_index = 0;
double gyroYaw = 0;
double accelYaw = 0;
double complementaryYaw = 0;
double yawITerm = 0;
double initHeading = 0;
double lastHeading = 0;
double gyroTotal = 0;
bool motorsOff = false;
// Four corner positions.  
//Back left is 0,0.  
//Back right is 0, 8.
//Front left is 8, 0
//Front right is 8,8
//3.6 x 4.2 y
//4.2 x -5.6 y
// -5.9x -6.2 y
//- 6x 3.8 y


//4.01, -5.39
// -5.93,  -6.09
// -5.95, 3.76
//3.6 4.0
// Old values:

double xInit1 = -6.2;
double xInit2 = -5.9;
double xMax1 = 3.6;
double xMax2 = 4.2;
double yInit1 = -6.2;
double yInit2 = -5.6;
double yMax1 = 4.2;
double yMax2 = 3.8;

//new:
/*
double xInit1 = -5.9;
double xInit2 = -6.01;
double xMax1 = 4.20;
double xMax2 = 3.61;
double yInit1 = -6.21;
double yInit2 = -5.62;
double yMax1 = 4.11;
double yMax2 = 3.8;
*/
//these pin #s were chosen randomly.  These are for h bridge stuff,
// make sure the A pins are actually PWM pins from teensy lc pinout
// Make sure that the d pins are preferably normal pins that don't have anlaog in
// A pins go to the analog input for the h bridge (enable)
// D pins go t othe digital input for H bridge (phase)
int frontLeftA = 22;
int frontLeftD = 16;
int backLeftA = 20;
int backLeftD = 14;

int frontRightA = 23;
int frontRightD = 17;
int backRightA = 21;
int backRightD = 15; 
bool slight = true;

short discLastX = -1;
short discLastY = -1;

//Setup the serial for the xbee
void xbeeSetup(){
  Serial1.begin(9600);
  lastEnemyTime = millis();
}

//setup the light to digital sensor(s?)
void ltdSetup(){
  pinMode(V1PIN, INPUT);
  V1.prevPulse = 0;
  V1.horzAng = 0;
  V1.vertAng = 0;
  V1.useMe = 0;
  V1.collected = 0;
  attachInterrupt(digitalPinToInterrupt(V1PIN), ISRV1, CHANGE);
  pinMode(V2PIN, INPUT); // to read the sensor
  // initialize the sensor variables
  V2.horzAng = 0;
  V2.vertAng = 0;
  V2.useMe = 0;
  V2.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V2PIN), ISRV2, CHANGE);
  pinMode(V3PIN, INPUT); // to read the sensor
  // initialize the sensor variables
  V3.horzAng = 0;
  V3.vertAng = 0;
  V3.useMe = 0;
  V3.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V3PIN), ISRV3, CHANGE);
  lastFilt1Time = millis();
  lastFilt2Time = millis();
  lastFilt3Time = millis();
  lightPoint initPoint;
  initPoint.x = -1999;
  initPoint.y = -1999;
  for (short i = 0; i < 30; ++i){
    filterPosArr[i] = initPoint;
  }
}


void imuSetup() 
{

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  while (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
  
  }
  imu.calibrate(true);
  imu.calibrateMag(true);
  lastYawTime = micros();
  double headingSum = 0;
  for (int i =0; i < 100; ++i){
    calcYaw();
    headingSum += lastHeading;
    delay(10);
  }
  initHeading = headingSum / 100.0;
  gyroOffset = gyroTotal / 100.0;
  gyroYaw = 0.0;
  calibGyro = false;
  
}

void imuReadVals(){
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}



void findPosition(double &xOld, double &yOld, double &xFilt, double &yFilt, short num){
      double xPos = 0;
      double yPos = 0;
      /*Serial.print("Num: \t");
      Serial.print(num);
      Serial.print("\r\n");*/
      if (num == 1){
        V1.useMe = 0;
        xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
      }
      else if (num == 2){
        V2.useMe = 0;
        xPos = tan((V2.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V2.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
      }
      else if (num == 3){
        Serial.println("IN 3");
        V3.useMe = 0;
        xPos = tan((V3.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V3.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
      }
      else if (num == 4){
       //v4.useMe = 0;
       //xPos = tan((V4.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
       //yPos = tan((V4.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
      }

 

      xFilt = xOld * 0.5 + xPos * 0.5;
      yFilt = yOld * 0.5 + yPos * 0.5;

      xOld = xFilt;
      yOld = yFilt;
}
double calcYaw(){
 imuReadVals();
 double heading;
 double mx, my;
 mx = imu.calcMag(imu.mx);
 my = imu.calcMag(imu.my);
  if (my == 0)
  {
    heading = (mx < 0) ? PI : 0;
  }
  else{
    heading = (double)  atan2(mx, my);
  }
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  //return heading;*/

  double gyroZ = imu.calcGyro(imu.gz);
  if (calibGyro){
    gyroTotal += gyroZ;
  }
  unsigned long tempTime= micros();
  gyroYaw += (gyroZ - gyroOffset) * (tempTime - lastYawTime) * .000001;
  lastYawTime = tempTime;
  lastHeading = heading;
  return fmod(gyroYaw, 360) * .9 + .1 * (heading - initHeading);
}

  


void getEnemyPosition(){  
  
    double tempX;
    double tempY;
   if (Serial1.available() > 0) {
   msg[msg_index] = Serial1.read();
   //Serial.print(msg[msg_index]);
   if (msg[msg_index] == '\n') {
     sscanf(msg, "%lf %lf", &tempX, &tempY);  
     msg_index = 0;
   }
   else {
     msg_index++;
     if (msg_index == 100) {
       msg_index = 0;
     }
   }
   lightPoint tempE;
   tempE.x = tempX;
   tempE.y = tempY;
    double summx = 0;
    double summy = 0;
    short numEnemies = 0;
    for(short i = 0; i < 10; ++i){
      if (enemyArr[i].x > - 100 && enemyArr[i].y > -100){
        summx += enemyArr[i].x;
        summy += enemyArr[i].y;
        ++numEnemies;
    }
   }
   if(abs(tempX - enemyX) < 1.5 || abs(tempY - enemyY) < 1.5 || ((millis() - lastEnemyTime) > 50)){
     enemyArr[enemyIndex] = tempE;
     ++enemyIndex;
     if(enemyIndex > 9){
      enemyIndex = 0;
     }
     lastEnemyTime = millis();
   }
   else{
    if (numEnemies > 0){
      enemyX = summx / numEnemies;
      enemyY = summy / numEnemies;
      return;
    }
    else{
      enemyX = 0.0;
      enemyY = 0.0;
    }
   }
  
 
   if (numEnemies > 0 ){
     enemyX = .7 * summx / numEnemies + .3 * tempX;
     enemyY = .7 * summy / numEnemies + .3 * tempY;
   }
   else{
    enemyX = tempX;
    enemyY = tempY;
   }
 }
 

}

double toDegrees(double rads){
  return rads * 180.0 / M_PI;
}



// should take it so it has arguments
void posToGrid(short &xCoord, short &yCoord, double xInGrid, double yInGrid){ 
  double xMaxSlope;
  double xMaxAvg = (xMax2 + xMax1 ) / 2.0;
  double xInitAvg = (xInit1 + xInit2) / 2.0;
  double yMaxAvg = (yMax2 + yMax1) / 2.0;
  double yInitAvg = (yInit1 + yInit2) / 2.0;
  double closeToX = (xInGrid - xInitAvg) / (xMaxAvg - xInitAvg);
  double closeToY = (yInGrid - yInitAvg) / (yMaxAvg - yInitAvg);
  double xMax = closeToY * (xMax2 - xMax1) + xMax1;
  double yMax = closeToX * (yMax2 - yMax1) + yMax1;
  double xInit = closeToY * (xInit2 - xInit1) + xInit1;
  double yInit = closeToX * (yInit2 - yInit1) + yInit1;
  double xPercent = (xInGrid - xInit) / (xMax - xInit);
  xCoord = round(xPercent * 8.0);
  double yPercent = (yInGrid - yInit) / (yMax - yInit);
  yCoord = round(yPercent * 8.0);
}


void gridToPos(short xCoord, short yCoord, double &xExact, double &yExact){
  double xMaxAvg = (xMax2 + xMax1 ) / 2.0;
  double xInitAvg = (xInit1 + xInit2) / 2.0;
  double yMaxAvg = (yMax2 + yMax1) / 2.0;
  double yInitAvg = (yInit1 + yInit2) / 2.0;
  double closeToX = (double(xCoord) + .5) / 9.0;
  double closeToY = (double(yCoord) +.5) / 9.0;
  double xMax = closeToY * (xMax2 - xMax1) + xMax1;
  double yMax = closeToX * (yMax2 - yMax1) + yMax1;
  double xInit = closeToY * (xInit2 - xInit1) + xInit1;
  double yInit = closeToX * (yInit2 - yInit1) + yInit1;
  double xDiff = xMax - xInit;
  double yDiff = yMax - yInit;
  xExact = closeToX * xDiff + xInit;
  yExact = closeToY * yDiff + yInit;
}

//read X and Y seperated by space.

//eventualy make initMotors() function
//left and right speeds between 0 and 255
//for directions True is forwards, False is backwards
void moveMotors(int leftSpeed, bool leftDirection, int rightSpeed, bool rightDirection) {
  analogWrite(frontLeftA, leftSpeed);
  analogWrite(backLeftA, leftSpeed);
  digitalWrite(frontLeftD, leftDirection);
  digitalWrite(backLeftD, leftDirection);
  analogWrite(frontRightA, rightSpeed);
  analogWrite(backRightA, rightSpeed);
  digitalWrite(frontRightD, rightDirection);
  digitalWrite(backRightD, rightDirection);

}

//setup the outputs for hbridge
void motorSetup() {
  // set the H bridge pins to all output mode
  pinMode(frontLeftA, OUTPUT);
  pinMode(frontLeftD, OUTPUT);
  pinMode(backLeftA, OUTPUT);
  pinMode(backLeftD, OUTPUT);
  pinMode(frontRightA, OUTPUT);
  pinMode(frontRightD, OUTPUT);
  pinMode(backRightA, OUTPUT);
  pinMode(backRightD, OUTPUT);
}


double distanceFunc(short distX, short distY, short otherX, short otherY){
  short xDiff = otherX - distX;
  short yDiff = otherY - distY;
  return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

bool onWall(short testX, short testY){
  return (testX == 0 || testX == 8 || testY == 0 || testY == 8);
}

// bigger is better
double newDesirability(short trialX, short trialY){
  //this is function that should definitely be talked about more and changed.
  return NEW_DISTANCE_WEIGHT * distanceFunc(trialX, trialY, theirCurrX, theirCurrY) - MY_DISTANCE_WEIGHT * distanceFunc(trialX, trialY, ourCurrX, ourCurrY) - NEW_WALL_WEIGHT * onWall(trialX, trialY);
}

void decideNewPos(short &goHereX, short &goHereY){
  double bestDesirability = -1;
  double tempDes;
  if(ourCurrX % 2 == 0){
    if (ourCurrY != 0){
      tempDes = newDesirability(ourCurrX, ourCurrY - 1);
      if (tempDes < bestDesirability){
        goHereX = ourCurrX;
        goHereY = ourCurrY - 1;
      }
    }
    if (ourCurrY != 8){
      tempDes = newDesirability(ourCurrX, ourCurrY + 1);
      if (tempDes < bestDesirability){
        goHereX = ourCurrX;
        goHereY = ourCurrY + 1;
      }
    }
  }
  if (ourCurrY % 2 == 0){
    if (ourCurrX != 0){
      tempDes = newDesirability(ourCurrX - 1, ourCurrY);
      if (tempDes < bestDesirability){
        goHereX = ourCurrX - 1;
        goHereY = ourCurrY;
      }
    }
    if (ourCurrX != 8){
      tempDes = newDesirability(ourCurrX + 1, ourCurrY);
      if (tempDes < bestDesirability){
        goHereX = ourCurrX + 1;
        goHereY = ourCurrY;
      }
    }
  }
}


void findPosition1(double &xOld, double &yOld, double &xFilt, double &yFilt, short num){
      double xPos = 0;
      double yPos = 0;
      /*Serial.print("Num: \t");
      Serial.print(num);
      Serial.print("\r\n");*/
      char tempIndex;
      if (num == 1){
        V1.useMe = 0;
        xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
       if(abs(xPos - xFilt1) < 1.5 || abs(yPos - yFilt1) < 1.5 || ((millis() - lastFilt1Time) > 50)){
         tempIndex = filter1PosIndex;
         ++filter1PosIndex;
         if (filter1PosIndex > 9){
           filter1PosIndex = 0;
         }
         lastFilt1Time = millis();
       }
       else{
        xPos = xFilt1;
        yPos = yFilt1;
       }
      }
      else if (num == 2){
        V2.useMe = 0;
        xPos = tan((V2.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V2.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
        if(abs(xPos - xFilt2) < 1.5 || abs(yPos - yFilt2) < 1.5 || ((millis() - lastFilt2Time) > 50)){
         tempIndex = filter2PosIndex;
         ++filter2PosIndex;
         if (filter2PosIndex > 9){
           filter2PosIndex = 0;
         }
         lastFilt2Time = millis();
       }
       else{
        xPos = xFilt2;
        yPos = yFilt2;
       }
      }
      else if (num == 3){
        V3.useMe = 0;
        xPos = tan((V3.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V3.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
        if(abs(xPos - xFilt3) < 1.5 || abs(yPos - yFilt3) < 1.5 || ((millis() - lastFilt3Time) > 50)){
         tempIndex = filter3PosIndex;
         ++filter3PosIndex;
         if (filter3PosIndex > 9){
           filter3PosIndex = 0;
         }
         lastFilt3Time = millis();
       }
       else{
        xPos = xFilt3;
        yPos = yFilt3;
       }
      }
      //set to 0 for array indexing.
      --num;
      
      
      double summx = 0;
      double summy = 0;
      double numPoints = 0;
      for (short tempIndex2 = (num * 10); tempIndex2 < (num + 1) * 10; ++tempIndex2){
        if (filterPosArr[tempIndex2].x > - 100 && filterPosArr[tempIndex2].y > -100){
          summx += filterPosArr[tempIndex2].x;
          summy += filterPosArr[tempIndex2].y;
          ++numPoints;
          }
      }
      lightPoint tempPoint;
      tempPoint.x = xPos;
      tempPoint.y = yPos;
      double xavg;
      double yavg;
      filterPosArr[num * 10 + tempIndex] = tempPoint;
       if (numPoints > 0 ){
        xavg = summx / numPoints;
        yavg = summy / numPoints;
       }
       else{
        xFilt = xPos;
        yFilt = yPos;
        return;
       }
       
       xFilt = .7 * xavg + .3 * xPos;
       yFilt = .7 * yavg + .3 * yPos;
        
        
      

     /* xFilt = xOld * 0.5 + xPos * 0.5;
      yFilt = yOld * 0.5 + yPos * 0.5;

      xOld = xFilt;
      yOld = yFilt;*/
}


// if either we or they change pos, do stuff 
bool needToRecalc(){
  return false;
 // if(ourLastX != g
}


void bind(short &val){
  if (val <0){
    val = 0;
  }
  if (val > 8){
    val = 8;
  }
}


void setup(){
  Serial.begin(9600);
  motorSetup();
  xbeeSetup();
  ltdSetup();
  Serial.println("enter precalc");
  preCalculateMatrix();
  Serial.println("Done calculating");
  imuSetup();
  

  
//  desiredYaw = 90;
}


void loop() {
  // call stuff in here
  // also if the directions are wrong, you can switch which wire goes to which out pin from the 
  // h bridge for that motor

    if (micros() - prevTime > 1000000 / 25) {
    if (V1.useMe == 1) {
      prevTime = micros();
      findPosition(xOld1, yOld1, xFilt1, yFilt1, 1);
    }
    }
    if (micros() - prevTime2 > 1000000 / 25){
      if (V2.useMe == 1){
        prevTime2 = micros();
        findPosition(xOld2, yOld2, xFilt2, yFilt2, 2);
    }
    }  
    if (micros() - prevTime2 > 1000000 / 25){
    if (V3.useMe == 1){
      prevTime3 = micros();
      findPosition(xOld3, yOld3, xFilt3, yFilt3, 3);
    }
    }
   
       
    
     getEnemyPosition();
 

    //calc average from lighthouse
    double xSum = xFilt3 + xFilt2 + xFilt1;
    double newXCombo = xSum / 3.0;
    double newYCombo = (double)(yFilt3 + yFilt2 + yFilt1) / 3.0;

    // eliminate outliers
    // :GENERAL STRAT:
    // calculate how far new x is away from last xcombo.  
    // if the idfference between one of lighthouse and other 2 is more than that diff,
    // then ignore it2
    // maybe calc sum and the # of divisors.  decide to subtract from sum and ALSO subtract from # of divisors.
    // Don't forget; if # of divisors is 0, then use old xCombo.   
 



    // after we have the finalized combos in newXCombo and newYCombo::
    // PUT IT INTO Grid woooo
    // Also saving into curr
    short goalY = ourCurrY;
    short goalX = ourCurrX + 1;
    short nextGoalX;
    short nextGoalY;
    // if its odd / in column
    bool turn = false;
    short ourTempLastX = ourCurrX;
    short ourTempLastY = ourCurrY;
    short theirTempLastX = theirCurrX;
    short theirTempLastY = theirCurrY;
    posToGrid(ourCurrX, ourCurrY, newXCombo, newYCombo);
    posToGrid(theirCurrX, theirCurrY, enemyX, enemyY);
    if (!(ourCurrX % 2 == 1 && ourCurrY % 2 == 1)){
      ourLastX = ourTempLastX;
      ourLastY = ourTempLastX;
    }
    else {
      ourCurrX = ourLastX;
      ourCurrY = ourLastY;
    }
    if (!(theirCurrX % 2 == 1 && theirCurrY %2 == 1)){
      theirLastX = theirTempLastX;
      theirLastY = theirTempLastY;
    }
    else{
      theirCurrX = theirLastX;
      theirCurrY = theirLastY;
    }
    bind(ourCurrX);
    bind(ourCurrY);
    bind(ourLastX);
    bind(ourLastY);
    bind(theirCurrX);
    bind(theirCurrY);
    bind(theirLastX);
    bind(theirLastY);
    //direction stuff:


    

    //ourCurrX, ourCurrY are our grid positions (shorts)
    //theirCurrX, theirCurrY are their grid positions (shorts)
    //call your function, and give me (for now) goalX and goalY (as shorts) for grid positions
    //nextGoalX and nextGoalY are the next ones (also shorts). 
    //All in grid positions, (0-8), (0-8)
   // decideNewPos(goalX, goalY); 
    int index;
    
    //CASEs for when we are not on even and they are not on even squares grid
    if (!(ourCurrX%2 == 0 && ourCurrY%2 == 0) && !(theirCurrX%2 == 0 && theirCurrY%2 == 0)){
      index = getIndex((int)(2*ourCurrX-ourLastX), (int)(2*ourCurrY-ourLastY), (int)(2*theirCurrX-theirLastX), (int)(2*theirCurrY-theirLastY));
    }
    else if (!(theirCurrX%2 == 0 && theirCurrY%2 == 0)){
      index = getIndex((int)ourCurrX, (int)ourCurrY, (int)(2*theirCurrX-theirLastX), (int)(2*theirCurrY-theirLastY));
    }
    else if(!(ourCurrX%2 == 0 && ourCurrY%2 == 0)){
      index = getIndex((int)(2*ourCurrX-ourLastX), (int)(2*ourCurrY-ourLastY), (int)theirCurrX, (int)theirCurrY);
    }
    else{
      index = getIndex((int)ourCurrX, (int)ourCurrY, (int)theirCurrX, (int)theirCurrY);
    }
    short tempX = goalX;
    short tempY = goalY;
    goalX = (short)PATHMATRIX[index][0].xpos;
    goalY = (short)PATHMATRIX[index][0].ypos;
    nextGoalX = (short)PATHMATRIX[index][1].xpos;
    nextGoalY = (short)PATHMATRIX[index][1].ypos;


    Serial.println("entered loop");

    double gridCenterX;
    double gridCenterY;
    gridToPos(ourCurrX, ourCurrY, gridCenterX, gridCenterY);
    
    if ((goalX > ourCurrX) && (abs(newYCombo - gridCenterY) < .15)){
      
      desiredYaw = 90;
      motorsOff = false;
    }
    else if ((goalX < ourCurrX) && (abs(newYCombo - gridCenterY) < .15)){
      desiredYaw = -90;
      motorsOff = false;
    }
    else if ((goalY > ourCurrY) && (abs(newXCombo - gridCenterX) < .15)){
      desiredYaw = 0;
      motorsOff = false;
    }
    else if ((goalY < ourCurrY) && (abs(newXCombo - gridCenterX < .15))){
      desiredYaw = 180;
      motorsOff = false;
    }
    else if ((goalX == ourCurrX) && goalY == ourCurrY) {
      motorsOff = true;
    }
    motorsOff = true;
    
    //imu stuff:
   // imuReadVals();
    // get yaw
    double gyroZ = imu.calcGyro(imu.gz);
    unsigned long tempTime= micros();
    gyroYaw += gyroZ * (tempTime - lastYawTime) * .000001;
    lastYawTime = tempTime;
 
    double yaw = calcYaw();
    double rightMotorSpeed;
    double leftMotorSpeed;
    // NEED TO DO THIS WITH THE 360 wrap around thing
    if (desiredYaw > 360){
      desiredYaw = fmod(desiredYaw, 360);
    }
    double diff = yaw - desiredYaw;
    if (diff > 180){
      diff = diff - 360;
    }
    else if (diff < -180){

    diff = 360 - abs(diff);
    }
    yawITerm += diff;
    double realDiff = diff;
    if (abs(realDiff) >25){
      slight = false;
    }
    if(slight){    
      rightMotorSpeed = NEUTRAL_POWER + (diff * slightkP) + (extremekI * yawITerm);
      leftMotorSpeed = NEUTRAL_POWER - (diff * slightkP) - (extremekI * yawITerm);
    
      if (leftMotorSpeed > 255){
        leftMotorSpeed = 255;
      }
      else if (leftMotorSpeed < 0){
        leftMotorSpeed = 0;
      }
      if (rightMotorSpeed > 255){
        rightMotorSpeed = 255;
      }
      else if (rightMotorSpeed < 0){
        rightMotorSpeed = 0;
      }
    if (!motorsOff){
      moveMotors(leftMotorSpeed, true, rightMotorSpeed, true);
    }
    }
    else if (abs(realDiff) > 1){
      
      rightMotorSpeed = EXTREME_NEUTRAL_POWER + (abs(realDiff) * extremekP) + (extremekI * yawITerm);;
      leftMotorSpeed = EXTREME_NEUTRAL_POWER + (abs(realDiff) * extremekP) - (extremekI * yawITerm);;
      if (leftMotorSpeed > 255){
        leftMotorSpeed = 255;
      }
      else if (leftMotorSpeed < 0){
        leftMotorSpeed = 0;
      }
      if (rightMotorSpeed > 255){
        rightMotorSpeed = 255;
      }
      else if (rightMotorSpeed < 0){
        rightMotorSpeed = 0;
      }
      // if we turn left:
      if (realDiff > 0){
        if (!motorsOff){
          moveMotors(leftMotorSpeed, false, rightMotorSpeed, true);   
        }    
      }
      // if we go right tho
      else{
        if (!motorsOff){
         moveMotors(leftMotorSpeed, true, rightMotorSpeed, false); 
        }
       }
    }
    else{
      slight = true;
    }
   
    //print stuff
    
    Serial.print("Xfilt1: \t");
    Serial.print(xFilt1);
    Serial.print("\t");
    Serial.print("Yfilt1: \t");
    Serial.print(yFilt1);
    Serial.print("\r\n");
    Serial.print("Xfilt2: \t");
    Serial.print(xFilt2);
    Serial.print("\t");
    Serial.print("Yfilt2: \t");
    Serial.print(yFilt2);
    Serial.print("\r\n");  
    Serial.print("Xfilt3: \t");
    Serial.print(xFilt3);
    Serial.print("\t");
    Serial.print("Yfilt3: \t");
    Serial.print(yFilt3);
    Serial.print("\r\n");
    /*
    Serial.print("Enemy xPos: \t");
    Serial.print(enemyX);
    Serial.print("\t");
    Serial.print("Enemy yPos: \t");
    Serial.print(enemyY);
    Serial.print("\r\n");
    
    Serial.print("Yaw: \t");
    Serial.print(yaw);
    Serial.print("\r\n");
    Serial.print("Desired yaw \t");
    Serial.print(desiredYaw);
    Serial.print("\r\n");
    Serial.print(diff);
    Serial.print("\r\n");
    Serial.print("LEft Power: \t");
    Serial.print(leftMotorSpeed);
    Serial.print("\r\n");
    Serial.print("Right motor power: \t");
    Serial.print(rightMotorSpeed);
    Serial.print("\r\n");
 */
    Serial.print("Xcombo: \t");
    Serial.println(newXCombo);
    Serial.print("Ycombo: \t");
    Serial.println(newYCombo); 
    Serial.print("X grid \t");
    Serial.println(ourCurrX);
    Serial.print("Y grid \t");
    Serial.println(ourCurrY);
    
    Serial.print("Theri X \t");
    Serial.println(theirCurrX);
    Serial.print("Their Y \t");
    Serial.println(theirCurrY);
    Serial.print("grid Center X: \t");
    Serial.println(gridCenterX);
    Serial.print("grid Center Y: \t");
    Serial.println(gridCenterY);
   

}
void ISRV1() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V1.changeTime[i] = V1.changeTime[i + 1];
  }
  V1.changeTime[10] = mic;

  // if the buffer is full
  if (V1.collected < 11) {
    V1.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V1.changeTime[1] - V1.changeTime[0] > 7000) && (V1.changeTime[3] - V1.changeTime[2] > 7000) && (V1.changeTime[6] - V1.changeTime[5] < 50) && (V1.changeTime[10] - V1.changeTime[9] < 50)) {
      V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_US;
      V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_US;
      V1.useMe = 1;
    }
  }
}


// the second ensor interrupt
void ISRV2() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V2.changeTime[i] = V2.changeTime[i + 1];
  }
  V2.changeTime[10] = mic;

  // if the buffer is full
  if (V2.collected < 11) {
    V2.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V2.changeTime[1] - V2.changeTime[0] > 7000) && (V2.changeTime[3] - V2.changeTime[2] > 7000) && (V2.changeTime[6] - V2.changeTime[5] < 50) && (V2.changeTime[10] - V2.changeTime[9] < 50)) {
      V2.horzAng = (V2.changeTime[5] - V2.changeTime[4]) * DEG_PER_US;
      V2.vertAng = (V2.changeTime[9] - V2.changeTime[8]) * DEG_PER_US;
      V2.useMe = 1;
    }
  }
}

//3rd sensor interrupt
void ISRV3() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V3.changeTime[i] = V3.changeTime[i + 1];
  }
  V3.changeTime[10] = mic;

  // if the buffer is full
  if (V3.collected < 11) {
    V3.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V3.changeTime[1] - V3.changeTime[0] > 7000) && (V3.changeTime[3] - V3.changeTime[2] > 7000) && (V3.changeTime[6] - V3.changeTime[5] < 50) && (V3.changeTime[10] - V3.changeTime[9] < 50)) {
      V3.horzAng = (V3.changeTime[5] - V3.changeTime[4]) * DEG_PER_US;
      V3.vertAng = (V3.changeTime[9] - V3.changeTime[8]) * DEG_PER_US;
      V3.useMe = 1;
    }
  }
}
