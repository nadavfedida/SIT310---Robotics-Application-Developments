#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1_left 10
#define SHT_LOX2_front 11
#define SHT_LOX3_right 12

// objects for the vl53l0x
Adafruit_VL53L0X lox1_left = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2_front = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3_right = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1_left, LOW);    
  digitalWrite(SHT_LOX2_front, LOW);
  digitalWrite(SHT_LOX3_right, LOW);

  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1_left, HIGH);
  digitalWrite(SHT_LOX2_front, HIGH);
  digitalWrite(SHT_LOX3_right, HIGH);

  delay(10);

  // activating LOX1 and resetting LOX2 and LOX3
  digitalWrite(SHT_LOX1_left, HIGH);
  digitalWrite(SHT_LOX2_front, LOW);
  digitalWrite(SHT_LOX3_right, LOW);


  // initing LOX1
  if(!lox1_left.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2_front, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2_front.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3_right, HIGH);
  delay(10);

  //initing LOX2
  if(!lox3_right.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_sensors() {
  
  lox1_left.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2_front.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3_right.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("3: "));
  if(measure3.RangeStatus != 4) {
    Serial.print(measure3.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1_left, OUTPUT);
  pinMode(SHT_LOX2_front, OUTPUT);
  pinMode(SHT_LOX3_right, OUTPUT);


  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1_left, LOW);
  digitalWrite(SHT_LOX2_front, LOW);
  digitalWrite(SHT_LOX3_right, LOW);


  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();
 
}

void loop() {
   
  read_sensors();
  delay(100);
}
