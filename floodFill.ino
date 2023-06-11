//==============================
//      LIBRARIES
//==============================
#include <StackArray.h>
#include <Arduino.h>
#include <QueueArray.h>
#include "Adafruit_VL53L0X.h"
#define IMPLEMENTATION LIFO

//==============================
//      PIN DECLARATION
//==============================
#define LeftMotFwd 6  // Motor Forward pin
#define LeftMotRev 7  // Motor Reverse pin
#define RightMotFwd 4 // Motor Forward pin
#define RightMotRev 5 // Motor Reverse pin

//    Address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

//    Set the pins to shutdown
#define SHT_LOX1 10
#define SHT_LOX2 11
#define SHT_LOX3 12

//==============================
//      CREATE SENSOR OBJECTS
//==============================
//    Objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

//    This holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

//==============================
//      CREATE STRUCT FOR NODES
//==============================
struct Location
{
  int x;
  int y;
  int dist;
  bool N;
  bool E;
  bool S;
  bool W;
};
struct GridCell
{
  int x, y;
};

//==============================
//      VARIABLES
//==============================
int cellWidth = 4000;
int startingCellMovement = 1400;
int normal_rmp = 150;
int fast_rpm = 160;
int turnTicks = 900;
int count = 0;
const int mazesize = 4;
const int NUM_LOCATIONS = mazesize * mazesize;
Location locations[NUM_LOCATIONS];
StackArray<Location> stack;

const int GRID_SIZE = mazesize;
bool grid[GRID_SIZE][GRID_SIZE][4]; // 4 variables for N, E, S, W
GridCell gridcell[NUM_LOCATIONS];

// StackArray <GridCell> gridQueue;
int distance[mazesize][mazesize];
bool visited[mazesize][mazesize];

//  Queue<GridCell> gridQueue(); // Max 10 chars in this queue
//  Queue<GridCell> gridQueue = Queue<GridCell>();
QueueArray<GridCell> gridQueue;
QueueArray<int> temp;

// Initialising starting position
int x_loc = 0;
int y_loc = 0;

//  Pins for motor controls
//    RIGHT MOTOR
int RightencoderPin1 = 2;            // Encoder Output 'A' must connected with intreput pin of arduino.
int RightencoderPin2 = 3;            // Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int RightlastEncoded = 0;   // Here updated value of encoder store.
volatile long RightencoderValue = 0; // Raw encoder value
//    LEFT MOTOR
int LeftencoderPin1 = 8;            // Encoder Output 'A' must connected with intreput pin of arduino.
int LeftencoderPin2 = 9;            // Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int LeftlastEncoded = 0;   // Here updated value of encoder store.
volatile long LeftencoderValue = 0; // Raw encoder value
//    TEMP VARIABLES FOR DISTANCE
int FsensorTemp, LsensorTemp, RsensorTemp;

//      MAZE INFO
const int maze_width = mazesize;
const int maze_height = mazesize;
const int finish_x = maze_width - 1;
const int finish_y = maze_height - 1;
//      COMPASS
const int north = 0;
const int east = 1;
const int south = 2;
const int west = 3;
//      ORENTATION
int direction;
//      OTHER
int decision;
//    WALL DISTANCE PARAMETER
int wallDistance = 220;
//    WALLS NEAR BY
bool leftWall = false;
bool frontWall = false;
bool rightWall = false;

//==============================
//      SET EACH SENSOR ID
//==============================
void setID()
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);

  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX2
  if (!lox2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // initing LOX2
  if (!lox3.begin(LOX3_ADDRESS))
  {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1)
      ;
  }
}

//==============================
//      UPDATE RIGHT ENCODER
//==============================
void updateRightEncoder()
{
  int MSB = digitalRead(RightencoderPin1); // MSB = most significant bit
  int LSB = digitalRead(RightencoderPin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;              // converting the 2 pin value to single number
  int sum = (RightlastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    RightencoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    RightencoderValue++;

  RightlastEncoded = abs(encoded); // store this value for next time
}

//==============================
//      UPDATE LEFT ENCODER
//==============================
void updateLeftEncoder()
{
  int MSB = digitalRead(LeftencoderPin1); // MSB = most significant bit
  int LSB = digitalRead(LeftencoderPin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;             // converting the 2 pin value to single number
  int sum = (LeftlastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    LeftencoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    LeftencoderValue++;

  LeftlastEncoded = abs(encoded); // store this value for next time
}

//==============================
//      CREATE NODES
//==============================
void initDistances()
{
  int n = 0;
  int m = 0;
  for (n = 0; n < mazesize; n++)
  {
    for (m = 0; m < mazesize; m++)
    {
      distance[n][m] = 0;
    }
  }
}

//==============================
//      RESET VISITED CELLS
//==============================
void resetVisited()
{
  int n = 0;
  int m = 0;
  for (n = 0; n < mazesize; n++)
  {
    for (m = 0; m < mazesize; m++)
    {
      visited[n][m] = false;
    }
  }
}

//==============================
//      PRINT NODES
//==============================
void printData()
{
  // Iterate over the array
  for (int i = 0; i < NUM_LOCATIONS; i++)
  {
    Serial.print("Location ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(locations[i].x);
    Serial.print(", ");
    Serial.print(locations[i].y);
    Serial.print("), N=");
    Serial.print(locations[i].N);
    Serial.print(", E=");
    Serial.print(locations[i].E);
    Serial.print(", S=");
    Serial.print(locations[i].S);
    Serial.print(", W=");
    Serial.println(locations[i].W);
  }
}

//==============================
//      INITIALISE GRID
//==============================
void initGrid()
{
  // Initialize the grid with all False values
  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      grid[i][j][0] = false; // N
      grid[i][j][1] = false; // E
      grid[i][j][2] = false; // S
      grid[i][j][3] = false; // W
    }
  }
}

//==============================
//      PRINT DISTANCES
//==============================
void printDistances()
{
  int X = x_loc;
  int Y = y_loc;
  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      Serial.print(distance[i][j]);
      if (i == X && j == Y)
      {
        Serial.print("*");
      }
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.print("Direction: ");
  Serial.println(direction);
}

//==============================
//      IS VALID CELL WITHIN BOUNDS
//==============================
bool inBounds(int X, int Y)
{
  if (X < 0 || X > mazesize - 1 || Y < 0 || Y > mazesize - 1)
  {
    return false;
  }
  else
    return true;
}

//==============================
//      SET PARAMETER WALL
//==============================
void updateWall(int X, int Y, int dir)
{
  //    NORTH
  grid[X][Y][dir] = true;
  if (dir == north && inBounds(X - 1, Y))
  {
    grid[X - 1][Y][south] = true;
  }
  //    EAST
  else if (dir == east && inBounds(X, Y + 1))
  {
    grid[X][Y + 1][west] = true;
  }
  //    SOUTH
  else if (dir == south && inBounds(X + 1, Y))
  {
    grid[X + 1][Y][north] = true;
  }
  //    WEST
  else if (dir == west && inBounds(X, Y - 1))
  {
    grid[X][Y - 1][east] = true;
  }
}

//==============================
//      SET WALL
//==============================
void setWalls()
{
  //    SET GLOBAL VARIABLES AS LOCALS
  int X = x_loc;
  int Y = y_loc;

  Serial.print("X: ");
  Serial.print(X);
  Serial.print("\tY: ");
  Serial.println(Y);

  switch (direction)
  {
  //    FACING NORTH
  case north:
    if (leftWall)
    {
      grid[X][Y][west] = true;
      updateWall(x_loc, y_loc, west);
    }
    if (frontWall)
    {
      grid[X][Y][north] = true;
      updateWall(x_loc, y_loc, north);
    }
    if (rightWall)
    {
      grid[X][Y][east] = true;
      updateWall(x_loc, y_loc, east);
    }
    break;

  //    FACING EAST
  case east:

    if (leftWall)
    {
      grid[X][Y][north] = true;
      updateWall(x_loc, y_loc, north);
    }
    if (frontWall)
    {
      grid[X][Y][east] = true;
      updateWall(x_loc, y_loc, east);
    }
    if (rightWall)
    {
      grid[X][Y][south] = true;
      updateWall(x_loc, y_loc, south);
    }
    break;

  //    FACING SOUTH
  case south:
    if (leftWall)
    {
      grid[X][Y][east] = true;
      updateWall(x_loc, y_loc, east);
    }
    if (frontWall)
    {
      grid[X][Y][south] = true;
      updateWall(x_loc, y_loc, south);
    }
    if (rightWall)
    {
      grid[X][Y][west] = true;
      updateWall(x_loc, y_loc, west);
    }
    break;

  //    FACING WEST
  case west:
    if (leftWall)
    {
      grid[X][Y][east] = true;
      updateWall(x_loc, y_loc, south);
    }
    if (frontWall)
    {
      grid[X][Y][south] = true;
      updateWall(x_loc, y_loc, west);
    }
    if (rightWall)
    {
      grid[X][Y][west] = true;
      updateWall(x_loc, y_loc, north);
    }
    break;

  default:
    break;
  }
}

//==============================
//      SET PARAMETER WALL
//==============================
// Set some values to True
void initWalls()
{
  for (int i = 0; i <= mazesize - 1; i++)
  {
    for (int j = 0; j <= mazesize - 1; j++)
    {
      // NORTH CASES
      if (i == 0)
        updateWall(i, j, 0);
      // SOUTH CASES
      if (i == mazesize - 1)
        updateWall(i, j, 2);
      // EAST CASES
      if (j == mazesize - 1)
        updateWall(i, j, 1);
      // WEST CASES
      if (j == 0)
        updateWall(i, j, 3);
    }
  }
  Serial.println("WALLS SET");

  // updateWall(0, 0, 2);
  // updateWall(0, 1, 2);
  // updateWall(1, 2, 1);
  // updateWall(1, 2, 2);
  // updateWall(1, 3, 2);
  // updateWall(2, 0, 1);
  // updateWall(3, 0, 1);
  // updateWall(3, 2, 0);
  // updateWall(3, 2, 1);
}

//==============================
//      FLOOD FILL
//==============================
void floodFill()
{
  int x = mazesize - 1;
  int y = mazesize - 1;
  resetVisited();
  gridQueue.enqueue({x, y});

  while (!gridQueue.isEmpty())
  {
    GridCell loc = gridQueue.dequeue();
    if (!visited[loc.x][loc.y])
    {
      visited[loc.x][loc.y] = true;

      //    CHECK NORTH
      if (!grid[loc.x][loc.y][0] && !visited[loc.x - 1][loc.y] && inBounds(loc.x - 1, loc.y))
      {
        distance[loc.x - 1][loc.y] = distance[loc.x][loc.y] + 1;
        gridQueue.enqueue({loc.x - 1, loc.y});
      }
      //    CHECK EAST
      if (!grid[loc.x][loc.y][1] && !visited[loc.x][loc.y + 1] && inBounds(loc.x, loc.y + 1))
      {
        distance[loc.x][loc.y + 1] = distance[loc.x][loc.y] + 1;
        gridQueue.enqueue({loc.x, loc.y + 1});
      }
      //    CHECK SOUTH
      if (!grid[loc.x][loc.y][2] && !visited[loc.x + 1][loc.y] && inBounds(loc.x + 1, loc.y))
      {
        distance[loc.x + 1][loc.y] = distance[loc.x][loc.y] + 1;
        gridQueue.enqueue({loc.x + 1, loc.y});
      }
      //    CHECK WEST
      if (!grid[loc.x][loc.y][3] && !visited[loc.x][loc.y - 1] && inBounds(loc.x, loc.y - 1))
      {
        distance[loc.x][loc.y - 1] = distance[loc.x][loc.y] + 1;
        gridQueue.enqueue({loc.x, loc.y - 1});
      }
    }
  }
}

//==============================
//      CHECK IF FINISHED MAZE
//==============================
void CheckIfFinished()
{
  if (x_loc == mazesize - 1 && y_loc == mazesize - 1)
  {
    Serial.println("FINISHED");
    delay(100000);
  }
}

//=====================================
//      CHECK ALL SURROUNDING WALLS
//=====================================
void CheckWalls()
{
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

  //    Check sensor one reading
  if (measure1.RangeStatus != 4)
  { // if not out of range
    LsensorTemp = measure1.RangeMilliMeter;
    Serial.print("L: ");
    Serial.print(LsensorTemp);
  }
  else
  {
    LsensorTemp = 300;
  }

  //    Check sensor two reading
  if (measure2.RangeStatus != 4)
  {
    FsensorTemp = measure2.RangeMilliMeter;
    Serial.print("\tF: ");
    Serial.print(FsensorTemp);
  }
  else
  {
    FsensorTemp = 300;
  }

  //    Check sensor three reading
  if (measure3.RangeStatus != 4)
  {
    RsensorTemp = measure3.RangeMilliMeter;
    Serial.print("\tR: ");
    Serial.println(RsensorTemp);
  }
  else
  {
    RsensorTemp = 300;
  }
  //    Set walls to true or false if they are there
  if (LsensorTemp < wallDistance)
  {
    leftWall = true;
  }
  if (FsensorTemp < wallDistance)
  {
    frontWall = true;
  }
  if (RsensorTemp < wallDistance)
  {
    rightWall = true;
  }
  Serial.print("L: ");
  Serial.print(leftWall);
  Serial.print("\tF: ");
  Serial.print(frontWall);
  Serial.print("\tR: ");
  Serial.println(rightWall);
}

//==============================
//       DECISIONS
//==============================
void decisions()
{
  // go to the cell with the smallest value
  int up, right, down, left;
  int X = x_loc;
  int Y = y_loc;
  int temp1 = 0;

  switch (direction)
  {
  // ====== NORTH =======
  case north:
    //    west
    if (inBounds(X, Y - 1) == 1 && grid[X][Y][west] == 0)
    {
      left = distance[X][Y - 1];
    }
    else
      left = 1000;
    //    north
    if (inBounds(X - 1, Y) == 1 && grid[X][Y][north] == 0)
    {
      up = distance[X - 1][Y];
    }
    else
      up = 1000;
    // east
    if (inBounds(X, Y + 1) == 1 && grid[X][Y][east] == 0)
    {
      right = distance[X][Y + 1];
    }
    else
      right = 1000;

    Serial.print("*** WALLS >> N: ");
    Serial.print(grid[X][Y][north]);
    Serial.print(", E: ");
    Serial.print(grid[X][Y][east]);
    Serial.print(", S: ");
    Serial.print(grid[X][Y][south]);
    Serial.print(", W: ");
    Serial.println(grid[X][Y][west]);

    Serial.print("UP: ");
    Serial.print(up);
    Serial.print(", RIGHT: ");
    Serial.print(right);
    Serial.print(", LEFT: ");
    Serial.println(left);
    //    SELECT MOVE
    if (up + left + right == 3000)
    {
      Uturn();
      delay(1000);
      forwards();
    }
    else if (up <= right && up <= left)
    {
      forwards();
    }
    else if (left < up && left < right)
    {
      turnLeft();
      delay(1000);
      forwards();
    }
    else if (right < up && right < left)
    {
      turnRight();
      delay(1000);
      forwards();
    }
    break;

  // ====== EAST =======
  case east:
    //    north
    if (inBounds(X - 1, Y) == 1 && grid[X][Y][north] == 0)
    {
      up = distance[X - 1][Y];
    }
    else
      up = 1000;
    // east
    if (inBounds(X, Y + 1) == 1 && grid[X][Y][east] == 0)
    {
      right = distance[X][Y + 1];
    }
    else
    {
      right = 1000;
    }
    // south
    if (inBounds(X + 1, Y) == 1 && grid[X][Y][south] == 0)
    {
      down = distance[X + 1][Y];
    }
    else
      down = 1000;

    Serial.print("*** WALLS >> N: ");
    Serial.print(grid[X][Y][north]);
    Serial.print(", E: ");
    Serial.print(grid[X][Y][east]);
    Serial.print(", S: ");
    Serial.print(grid[X][Y][south]);
    Serial.print(", W: ");
    Serial.println(grid[X][Y][west]);

    Serial.print("UP: ");
    Serial.print(up);
    Serial.print(", RIGHT: ");
    Serial.print(right);
    Serial.print(", DOWN: ");
    Serial.println(down);

    //    SELECT MOVE
    if (up + right + down == 3000)
    {
      Serial.println("1");
      Uturn();
      delay(1000);
      forwards();
    }
    else if (right <= up && right <= down)
    {
      Serial.println("2");
      forwards();
    }
    else if (up < right && up < down)
    {
      Serial.println("3");
      turnLeft();
      delay(1000);
      forwards();
    }
    else if (down < right && down < up)
    {
      Serial.println("4");
      turnRight();
      delay(1000);
      forwards();
    }
    break;

  // ====== SOUTH =======
  case south:
    // east
    if (inBounds(X, Y + 1) == 1 && grid[X][Y][east] == 0)
    {
      right = distance[X][Y + 1];
    }
    else
      right = 1000;
    // south
    if (inBounds(X + 1, Y) == 1 && grid[X][Y][south] == 0)
    {
      down = distance[X + 1][Y];
    }
    else
      down = 1000;
    //    west
    if (inBounds(X, Y - 1) == 1 && grid[X][Y][west] == 0)
    {
      left = distance[X][Y - 1];
    }
    else
      left = 1000;

    Serial.print("*** WALLS >> N: ");
    Serial.print(grid[X][Y][north]);
    Serial.print(", E: ");
    Serial.print(grid[X][Y][east]);
    Serial.print(", S: ");
    Serial.print(grid[X][Y][south]);
    Serial.print(", W: ");
    Serial.println(grid[X][Y][west]);

    Serial.print("LEFT: ");
    Serial.print(left);
    Serial.print(", RIGHT: ");
    Serial.print(right);
    Serial.print(", DOWN: ");
    Serial.println(down);

    //    SELECT MOVE
    if (left + right + down == 3000)
    {
      Uturn();
      delay(1000);
      forwards();
    }
    else if (down <= left && down <= right)
    {
      forwards();
    }
    else if (left < down && left < right)
    {
      turnRight();
      delay(1000);
      forwards();
    }
    else if (right < down && right < left)
    {
      turnLeft();
      delay(1000);
      forwards();
    }
    break;

  // ====== WEST =======
  case west:
    // south
    if (inBounds(X + 1, Y) == 1 && grid[X][Y][south] == 0)
    {
      down = distance[X + 1][Y];
    }
    else
      down = 1000;
    //    west
    if (inBounds(X, Y - 1) == 1 && grid[X][Y][west] == 0)
    {
      left = distance[X][Y - 1];
    }
    else
      left = 1000;
    //    north
    if (inBounds(X - 1, Y) == 1 && grid[X][Y][north] == 0)
    {
      up = distance[X - 1][Y];
    }
    else
      up = 1000;

    Serial.print("*** WALLS >> N: ");
    Serial.print(grid[X][Y][north]);
    Serial.print(", E: ");
    Serial.print(grid[X][Y][east]);
    Serial.print(", S: ");
    Serial.print(grid[X][Y][south]);
    Serial.print(", W: ");
    Serial.println(grid[X][Y][west]);

    Serial.print("UP: ");
    Serial.print(up);
    Serial.print(", LEFT: ");
    Serial.print(left);
    Serial.print(", DOWN: ");
    Serial.println(down);

    //    SELECT MOVE
    if (left + up + down == 3000)
    {
      Uturn();
      delay(1000);
      forwards();
    }
    else if (left <= up && left <= down)
    {
      forwards();
    }
    else if (up < down && up < left)
    {
      turnRight();
      delay(1000);
      forwards();
    }
    else if (down < left && down < up)
    {
      turnLeft();
      delay(1000);
      forwards();
    }
    break;

  default:
    break;
  }
}

//==============================
//      PRINT STACK
//==============================
void printStack()
{
  while (!stack.isEmpty())
  {
    Location currentLocation = stack.pop();
    Serial.print("Location (");
    Serial.print(currentLocation.x);
    Serial.print(", ");
    Serial.print(currentLocation.y);
    Serial.print("), N=");
    Serial.print(currentLocation.N);
    Serial.print(", E=");
    Serial.print(currentLocation.E);
    Serial.print(", S=");
    Serial.print(currentLocation.S);
    Serial.print(", W=");
    Serial.println(currentLocation.W);
  }
}
//==============================
//      MOVE: FORWARDS
//==============================
void forwards()
{
  Serial.println("<<< FORWARDS >>>");
  RightencoderValue = 0;
  LeftencoderValue = 0;
  while (abs(RightencoderValue) < cellWidth && abs(LeftencoderValue) < cellWidth)
  {
    printEncoders();
    if (abs(RightencoderValue) > abs(LeftencoderValue))
    {
      fowardLeftFast();
    }
    else if (abs(RightencoderValue) < abs(LeftencoderValue))
    {
      forwardRightFast();
    }
    else
    {
      moveForwards();
    }
    delay(50);
  }
  stop();
  updateCurrentCell();
}

//==============================
//      TURN: RIGHT
//==============================
void turnRight()
{
  Serial.println("<<< RIGHT >>>");
  RightencoderValue = 0;
  LeftencoderValue = 0;
  while (abs(RightencoderValue) < turnTicks && abs(LeftencoderValue) < turnTicks)
  {
    printEncoders();

    analogWrite(RightMotFwd, 0);
    analogWrite(RightMotRev, normal_rmp);
    analogWrite(LeftMotFwd, normal_rmp);
    analogWrite(LeftMotRev, 0);
    delay(50);
  }
  stop();

  //    Update direction after right turn
  updateDirection(90);
}

//==============================
//      TURN: LEFT
//==============================
void turnLeft()
{
  Serial.println("<<< LEFT >>>");
  RightencoderValue = 0;
  LeftencoderValue = 0;

  while (abs(RightencoderValue) < turnTicks && abs(LeftencoderValue) < turnTicks)
  {
    printEncoders();
    analogWrite(RightMotFwd, normal_rmp);
    analogWrite(RightMotRev, 0);
    analogWrite(LeftMotFwd, 0);
    analogWrite(LeftMotRev, normal_rmp);
    delay(50);
  }
  stop();

  //  Update direction after left turn
  updateDirection(270);
}

//==============================
//      TURN: Uturn
//==============================
void Uturn()
{
  Serial.println("<<< U-TURN >>>");
  RightencoderValue = 0;
  LeftencoderValue = 0;

  while (abs(RightencoderValue) < turnTicks * 2.5 && abs(LeftencoderValue) < turnTicks * 2.5)
  {
    printEncoders();
    analogWrite(RightMotFwd, normal_rmp);
    analogWrite(RightMotRev, 0);
    analogWrite(LeftMotFwd, 0);
    analogWrite(LeftMotRev, normal_rmp);
    delay(20);
  }
  stop();
  //    Update direction after u-turn
  updateDirection(180);
  delay(1000);
  realignment();
}

//==============================
//      REALIGNMENT MANUVOUR
//==============================
void realignment()
{
  Serial.println("<<< realignment >>>");
  Serial.println("MOVING BACK");
  RightencoderValue = 0;
  LeftencoderValue = 0;

  //     MOVE BACK INTO WALL TO SELF ALIGN
  while (abs(RightencoderValue) < startingCellMovement && abs(LeftencoderValue) < startingCellMovement)
  {
    printEncoders();
    if (abs(RightencoderValue) > abs(LeftencoderValue))
    {
      backwardsRLeftFast();
    }
    else if (abs(RightencoderValue) < abs(LeftencoderValue))
    {
      backwardsRightFast();
    }
    else
    {
      moveBackwards();
    }
    delay(50);
  }
  stop();

  Serial.println("MOVING TO THE MIDDLE");

  //     MOVE FORWARDS INTO CELL CENTRE TO SELF ALIGN
  delay(1000);
  halfStep();

  RightencoderValue = 0;
  LeftencoderValue = 0;
  //    Change current cell values
  // updateCurrentCell();
}

//==============================
//      HALF STEP
//==============================
void halfStep()
{
  Serial.println("<<< HALF STEP >>>");
  RightencoderValue = 0;
  LeftencoderValue = 0;
  while (abs(RightencoderValue) < startingCellMovement && abs(LeftencoderValue) < startingCellMovement)
  {
    printEncoders();
    if (abs(RightencoderValue) > abs(LeftencoderValue))
    {
      fowardLeftFast();
    }
    else if (abs(RightencoderValue) < abs(LeftencoderValue))
    {
      forwardRightFast();
    }
    else
    {
      moveForwards();
    }
    delay(50);
  }
  stop();
}

//==============================
//      UPDATE CURRENT LOCATIONS
//==============================
void updateCurrentCell()
{
  switch (direction)
  {
  case north:
    x_loc = x_loc - 1;
    break;

  case east:
    y_loc = y_loc + 1;
    break;

  case south:
    x_loc = x_loc + 1;
    break;

  case west:
    y_loc = y_loc - 1;
    break;

  default:
    break;
  }
}

//==============================
//      UPDATE DIRECTION
//==============================
void updateDirection(int angle)
{
  switch (direction)
  {
  case north:
    switch (angle)
    {
    case 270:
      direction = west;
      break;
    case 90:
      direction = east;
      break;
    case 180:
      direction = south;
    default:
      break;
    }
    break;

  case east:
    switch (angle)
    {
    case 270:
      direction = north;
      break;
    case 90:
      direction = south;
      break;
    case 180:
      direction = west;
    default:
      break;
    }
    break;

  case south:
    switch (angle)
    {
    case 270:
      direction = east;
      break;
    case 90:
      direction = west;
      break;
    case 180:
      direction = north;
    default:
      break;
    }
    break;

  case west:
    switch (angle)
    {
    case 270:
      direction = north;
      break;
    case 90:
      direction = south;
      break;
    case 180:
      direction = east;
    default:
      break;
    }
    break;

  default:
    break;
  }
}

//==============================
//      FORWARD
//==============================
void moveForwards()
{
  analogWrite(RightMotFwd, normal_rmp);
  analogWrite(RightMotRev, 0);
  analogWrite(LeftMotFwd, normal_rmp);
  analogWrite(LeftMotRev, 0);
}

//==============================
//      BACKWARDS
//==============================
void moveBackwards()
{
  analogWrite(RightMotFwd, 0);
  analogWrite(RightMotRev, normal_rmp);
  analogWrite(LeftMotFwd, 0);
  analogWrite(LeftMotRev, normal_rmp);
}

//==============================
//      STOP
//==============================
void stop()
{
  analogWrite(RightMotFwd, 0);
  analogWrite(RightMotRev, 0);
  analogWrite(LeftMotFwd, 0);
  analogWrite(LeftMotRev, 0);
}

//==============================
//      LEFT FAST
//==============================
void fowardLeftFast()
{
  analogWrite(RightMotFwd, normal_rmp);
  analogWrite(RightMotRev, 0);
  analogWrite(LeftMotFwd, fast_rpm);
  analogWrite(LeftMotRev, 0);
}

//==============================
//      RIGHT FAST
//==============================
void forwardRightFast()
{
  analogWrite(RightMotFwd, fast_rpm);
  analogWrite(RightMotRev, 0);
  analogWrite(LeftMotFwd, normal_rmp);
  analogWrite(LeftMotRev, 0);
}

//==============================
//      BACKWARDS RIGHT FAST
//==============================
void backwardsRightFast()
{
  analogWrite(RightMotFwd, 0);
  analogWrite(RightMotRev, fast_rpm);
  analogWrite(LeftMotFwd, 0);
  analogWrite(LeftMotRev, normal_rmp);
}

//==============================
//      BACKWARDS LEFT FAST
//==============================
void backwardsRLeftFast()
{
  analogWrite(RightMotFwd, 0);
  analogWrite(RightMotRev, normal_rmp);
  analogWrite(LeftMotFwd, 0);
  analogWrite(LeftMotRev, fast_rpm);
}

//==============================
//      PRINT ENCODER VALUES
//==============================
void printEncoders()
{
  // Serial.print("Encoders: L: ");
  // Serial.print(abs(LeftencoderValue));
  // Serial.print("\t R: ");
  // Serial.print(abs(RightencoderValue));
  // Serial.print("\t DIFF: ");
  // Serial.println(abs(RightencoderValue) - abs(LeftencoderValue));
}


//==============================
//      PRINT ALL WALLS
//==============================
void printAllWalls()
{


  for (int i = 0; i <= mazesize - 1; i++)
  {
    for (int j = 0; j <= mazesize - 1; j++)
    {
      Serial.print("(");
      Serial.print(i);
      Serial.print(",");
      Serial.print(j);
      Serial.print("), N:");
      Serial.print(grid[i][j][north]);
      Serial.print(", E: ");
      Serial.print(grid[i][j][east]);
      Serial.print(", S: ");
      Serial.print(grid[i][j][south]);
      Serial.print(", W: ");
      Serial.println(grid[i][j][west]);
    }
  }
}




//==============================
//      MAIN SETUP
//==============================
void setup()
{
  //    SERIAL
  Serial.begin(9600);
  Serial.print("+++++++++++++++++ START +++++++++++++++++");

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  setID();

  //    MOTORS
  pinMode(RightMotFwd, OUTPUT);
  pinMode(RightMotRev, OUTPUT);
  pinMode(LeftMotFwd, OUTPUT);
  pinMode(LeftMotRev, OUTPUT);

  pinMode(RightencoderPin1, INPUT_PULLUP);
  pinMode(RightencoderPin2, INPUT_PULLUP);
  pinMode(LeftencoderPin1, INPUT_PULLUP);
  pinMode(LeftencoderPin2, INPUT_PULLUP);

  digitalWrite(RightencoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(RightencoderPin2, HIGH); // turn pullup resistor on
  digitalWrite(LeftencoderPin1, HIGH);  // turn pullup resistor on
  digitalWrite(LeftencoderPin2, HIGH);  // turn pullup resistor on

  attachInterrupt(digitalPinToInterrupt(2), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(9), updateLeftEncoder, CHANGE);

  initDistances();
  initGrid();
  initWalls();
  floodFill();
  printDistances();

  //    SET INITIAL DIRECTION
  direction = east;
}

//==============================
//      MAIN LOOP
//==============================
int check = 0;
void loop()
{
  if (check == 0)
  {
    delay(3000);
    Serial.println("+++++++++++++++++ START +++++++++++++++++");
    initDistances();
    initGrid();
    initWalls();
    floodFill();
    printDistances();
    //    MOVE HALF CELL IN TO START AT MIDDLE OF (0,0)
    halfStep();
    delay(2000);
    // printAllWalls();
  }
  check++;

  CheckIfFinished();
  printDistances();
  CheckWalls();
  setWalls();
  floodFill();
  decisions();
  delay(2000);
  // Uturn();
  // delay(2000);
  // turnRight();
}