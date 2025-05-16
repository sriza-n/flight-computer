  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// amount the x and y positions are adjusted by on each itteration *** ADDED BY BEN CAUNT 
double xAdjust = 0;
double yAdjust = 0;

// these two values are compared to the processed acceleration values and if the sum of the acceleration values is within these two thresholds, it will NOT be added to the position

double MAXMIN_THRESH = 0.01;
double MINMIN_THRESH = -0.01;


double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

// thing that ben adde to scale the linear acceleration data to make the changes in position hopefully more dramatic
int ACCELERATION_SCALE_X = 125;//p original scale for X
int ACCELERATION_SCALE_Y = 100; // Use smaller scale for Y to reduce its influence

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(115200);
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }


  delay(1000);
}

void loop(void)
{
  // unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  // Get events from sensor (note: getEvent returns bool, not Vector)
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Now create vectors from the data
  imu::Vector<3> euler(orientationData.orientation.x, 
    orientationData.orientation.y, 
    orientationData.orientation.z);

  imu::Vector<3> linearAccel(linearAccelData.acceleration.x,
          linearAccelData.acceleration.y,
          linearAccelData.acceleration.z);
          
  // ********** added by Ben Caunt ****** this is what I am refering to in the email ***
  // essentially only add the value if the change is actually signifcant 
  // i believe what is occuring is potentially a floating point precision error in addition to just the simple fact that inertial navigation doesnt like sitting still
  // while this fix is certain not very elegant, it appears to be somewhat functional
  xAdjust = ACCEL_POS_TRANSITION * (linearAccelData.acceleration.x * ACCELERATION_SCALE_X);
  yAdjust = ACCEL_POS_TRANSITION * (linearAccelData.acceleration.y * ACCELERATION_SCALE_Y);
  if ((xAdjust + yAdjust > MAXMIN_THRESH) || (xAdjust + yAdjust < MINMIN_THRESH)) {
    xPos = xPos + xAdjust;
    yPos = yPos + yAdjust;
    
  }

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  // // Add z-axis heading velocity calculation
  // double zHeadingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.z;

  // // Apply pitch compensation (y rotation impacts z-axis measurements)
  // zHeadingVel = zHeadingVel / cos(DEG_2_RAD * orientationData.orientation.y);
  double totalAccel = sqrt(pow(linearAccel.x(), 2) + pow(linearAccel.y(), 2) + pow(linearAccel.z(), 2));
  //add threshold of 0.4 on total acceleration
  if (totalAccel < 0.4) {
    totalAccel = 0;
  }
  
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Head:");
    Serial.print(orientationData.orientation.x);
    Serial.print(",");
    Serial.print("Pos:");
    Serial.print(-xPos);
    //Serial.println(ACCEL_POS_TRANSITION * (linearAccelData.acceleration.x * ACCELERATION_SCALE));
    Serial.print(",");
    Serial.print(-yPos);
    //Serial.println(ACCEL_POS_TRANSITION * (linearAccelData.acceleration.y * ACCELERATION_SCALE));
    // Serial.print(",");
    // Serial.print("Speed:");
    // Serial.print(headingVel);
    Serial.print(",");
    Serial.print("eX:");
    Serial.print(euler.x());
    Serial.print(",");
    Serial.print("eY:");
    Serial.print(euler.y());
    Serial.print(",");
    Serial.print("eZ:");
    Serial.print(euler.z());
    Serial.print(",");
    Serial.print("aX:");
    Serial.print(linearAccel.x());
    Serial.print(",");
    Serial.print("aY:");
    Serial.print(linearAccel.y());
    Serial.print(",");
    Serial.print("aZ:");
    Serial.print(linearAccel.z());
    // Calculate total acceleration magnitude
    Serial.print(",");
    
// Add to serial output
    Serial.print("TotalAccel:");
    Serial.println(totalAccel);
    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }



  // while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  // {
  //   //poll until the next sample is ready
  // }
}
