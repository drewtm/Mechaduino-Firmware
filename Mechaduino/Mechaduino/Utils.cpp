//Contains the definitions of the functions used by the firmware.

#include <SPI.h>
#include <Wire.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"
#include "calibrations.h"

void setupPins() {

  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);

  pinMode(chipSelectPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
  
#ifdef ENABLE_PROFILE_IO  
  pinMode(TEST1, OUTPUT);
#endif

  pinMode(ledPin, OUTPUT); //

  // pinMode(clockPin, OUTPUT); // SCL    for I2C
  // pinMode(inputPin, INPUT); // SDA


  analogFastWrite(VREF_2, 0.33 * uMAX);
  analogFastWrite(VREF_1, 0.33 * uMAX);

  IN_4_HIGH();   //  digitalWrite(IN_4, HIGH);
  IN_3_LOW();    //  digitalWrite(IN_3, LOW);
  IN_2_HIGH();   //  digitalWrite(IN_2, HIGH);
  IN_1_LOW();    //  digitalWrite(IN_1, LOW);
}

void setupSPI() {
  SPISettings settingsA(10000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);
  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Beginning SPI communication with AS5047 encoder...");
  delay(1000);
  SPI.beginTransaction(settingsA);
}

void configureStepDir() {
  pinMode(step_pin, INPUT);
  pinMode(dir_pin, INPUT);
  attachInterrupt(step_pin, stepInterrupt, RISING);
  attachInterrupt(dir_pin, dirInterrupt, CHANGE);
}

void configureEnablePin() {
  pinMode(enable_pin, INPUT);
  attachInterrupt(enable_pin, enableInterrupt, CHANGE);
}

void stepInterrupt() {
  if (dir) r += stepangle;
  else r -= stepangle;
}

void dirInterrupt() {
  if (REG_PORT_IN0 & PORT_PA11) dir = false; // check if dir_pin is HIGH
  else dir = true;
}

void enableInterrupt() {            //enable pin interrupt handler
  if (REG_PORT_IN0 & PORT_PA14){   // check if enable_pin is HIGH
    disableTCInterrupts();
    analogFastWrite(VREF_2, 0);  //set phase currents to zero
    analogFastWrite(VREF_1, 0);
    }
  else{
    enableTCInterrupts();    
    }
}

//theta is the current position of the stepper, possibly with a phase offset to lead the stepper motion
void output(float theta, int effort, int phaseShift) {
  int angle_1;
  int angle_2;
  int v_coil_A;
  int v_coil_B;

  int sin_coil_A;
  int sin_coil_B;
  //convert from absolute angle to phase excitation angle, and from radians to fractions of a revolution (relative to the size of the sine table)
  const float phase_multiplier = (float(spr) / 4.0) * (float(sineTableSize) / (2.0 * PI));

  //REG_PORT_OUTCLR0 = PORT_PA09; for debugging/timing

  int signEffort = (effort<0)?(-1):(1);
  int calibrationTableOffset = (directionSwap>0)?0:sineTableSize/4;
  int intAngle = int(phase_multiplier * theta) - (signEffort*phaseShift) + calibrationTableOffset;
  effort = abs(effort);

  angle_1 = mod(intAngle, sineTableSize);
  angle_2 = mod(intAngle + (sineTableSize/4), sineTableSize);
  
  sin_coil_A  = sin_1[angle_1];

  sin_coil_B = sin_1[angle_2];

  v_coil_A = ((directionSwap * effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

     //For debugging:
//   SerialUSB.print(v_coil_A);
//   SerialUSB.print(" , ");
//   SerialUSB.println(v_coil_B);

  analogFastWrite(VREF_1, abs(v_coil_A));
  analogFastWrite(VREF_2, abs(v_coil_B));

  if (v_coil_A >= 0)  {
    IN_2_HIGH();  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
    IN_1_LOW();   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
  }
  else  {
    IN_2_LOW();   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
    IN_1_HIGH();  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
  }

  if (v_coil_B >= 0)  {
    IN_4_HIGH();  //REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
    IN_3_LOW();   //REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
  }
  else  {
    IN_4_LOW();     //REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
    IN_3_HIGH();    //REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
  }

}

float read_angle()
{
  const int avg = 10;            //average a few readings
  int encoderReading = 0;

  disableTCInterrupts();        //can't use readEncoder while in closed loop

  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    encoderReading += readEncoder();
    delay(10);
    }

  //return encoderReading * (2.0*PI / ENCODERCOUNTS) / avg;
  return lookup[encoderReading / avg];
}

void oneStep() {           /////////////////////////////////   oneStep    ///////////////////////////////
  if (dir) stepNumber++;
  else stepNumber--;
  output(aps * float(stepNumber), (int)(0.33 * uMAX));
  delay(10);
}

int readEncoder()           //////////////////////////////////////////////////////   READENCODER   ////////////////////////////
{
  long angleTemp;
  
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  return angleTemp;
}



void readEncoderDiagnostics()           //////////////////////////////////////////////////////   READENCODERDIAGNOSTICS   ////////////////////////////
{
  long angleTemp;
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
  SerialUSB.println("------------------------------------------------");

  SerialUSB.println("Checking AS5047 diagnostic and error registers");
  SerialUSB.println("See AS5047 datasheet for details");
  SerialUSB.println(" ");
  ;

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xC0);
  byte b2 = SPI.transfer(0x00);

  SerialUSB.print("Check DIAAGC register (0x3FFC) ...  ");
  SerialUSB.println(" ");

  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14))    SerialUSB.println("  Error occurred  ");

  if (angleTemp & (1 << 11))    SerialUSB.println("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased");

  if (angleTemp & (1 << 10))    SerialUSB.println("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased");

  if (angleTemp & (1 << 9))     SerialUSB.println("  COF - CORDIC overflow. This indicates the measured angle is not reliable");

  if (angleTemp & (1 << 8))     SerialUSB.println("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed");

  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  SerialUSB.println("Looks good!");
  SerialUSB.println(" ");


  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  SPI.transfer(0x40);
  SPI.transfer(0x01);
  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);

  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);


  SerialUSB.print("Check ERRFL register (0x0001) ...  ");
  SerialUSB.println(" ");



  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.println("  Error occurred  ");
  }
  if (angleTemp & (1 << 2)) {
    SerialUSB.println("  parity error ");
  }
  if (angleTemp & (1 << 1)) {
    SerialUSB.println("  invalid register  ");
  }
  if (angleTemp & (1 << 0)) {
    SerialUSB.println("  framing error  ");
  }
  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  SerialUSB.println("Looks good!");

  SerialUSB.println(" ");

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);


  delay(1);

}

void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    SerialUSB.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  SerialUSB.println(x);         // print the integer
  r = 0.1 * ((float)x);
}

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}


void setupTCInterrupts() {  // configure the controller interrupt

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / Fs)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable TC
  //  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  //  WAIT_TC16_REGS_SYNC(TC5)
}

void enableTCInterrupts() {   //enables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
}


void hybridControl() {        //still under development

  static int missed_steps = 0;
  static float iLevel = 0.6;  //hybrid stepping current level.  In this mode, this current is continuous (unlike closed loop mode). Be very careful raising this value as you risk overheating the A4954 driver!
  static float rSense = 0.15;

  if (yw < r - aps) {
    missed_steps -= 1;
  }
  else if (yw > r + aps) {
    missed_steps += 1;
  }

  output(0.1125 * (-(r - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));

}


void stepResponse() {     // not done yet...
  SerialUSB.println("");
  SerialUSB.println("--------------------------------");
  SerialUSB.println("");
  SerialUSB.println("Get ready for step response!");
  SerialUSB.println("Close Serial Monitor and open Tools>>Serial Plotter");
  SerialUSB.println("You have 10 seconds...");
  enableTCInterrupts();     //start in closed loop mode
  //mode = 'x';
  r = 0;
  delay(1000);
  SerialUSB.println("9...");
  delay(1000);
  SerialUSB.println("8...");
  delay(1000);
  SerialUSB.println("7...");
  delay(1000);
  SerialUSB.println("6...");
  delay(1000);
  SerialUSB.println("5...");
  delay(1000);
  SerialUSB.println("4...");
  delay(1000);
  SerialUSB.println("3...");
  delay(1000);
  SerialUSB.println("2...");
  delay(1000);
  SerialUSB.println("1...");
  delay(1000);
  print_yw = true;
  delay(100);
  r = 97.65;      /// choose step size as you like, 97.65 gives a nice plot since 97.65*1024 = 10,000
  delay(400);
  print_yw = false;
  r = 0;
  delay(500);
  disableTCInterrupts();
}



void moveRel(float pos_final,int vel_max, int accel){
  
   //Use this function for slow relative movements in closed loop position mode
   //
   // This function creates a "trapezoidal speed" trajectory (constant accel, and max speed, constant decel);
   // It works pretty well, but it may not be perfect
   // 
   // pos_final is the desired position in degrees
   // vel_max is the max velocity in degrees/second
   // accel is the max accel in degrees/second^2
   //
   //Note that the actual max velocity is limited by the execution speed of all the math below.
   //Adjusting dpos (delta position, or step size) allows you to trade higher speeds for smoother motion
   //Max speed with dpos = 0.225 degrees is about 180 deg/sec
   //Max speed with dpos = 0.45 degrees is about 360 deg/sec 
  
  float pos = 0;
  float dpos = 0.45*2.0*PI/360.0;  // "step size" in radians, smaller is smoother, but will limit max speed, keep below stepper step angle
  float vel = 0;      // 
  float vel_1 =0;
  int start = micros(); //for debugging

  float accel_x_dpos = accel*dpos;  // pre calculate
  float dpos_x_1000000 = dpos*1000000.0; // pre calculate

  float pos_remaining = pos_final-pos;
  unsigned long dt =0; 
  unsigned long t = micros();
  unsigned long t_1 = t;

  float r0 = r;  //hold initial setpoint

  // Assume we're decelerating and calculate speed along deceleration profile
  
  while (abs(pos_remaining) >(dpos/2)){  //(may not actually reach exactly so leave some margin
  
    if (pos_remaining > 0)        // clockwise
    vel = sqrt(2.0 * pos_remaining * accel);
    else                      // counter clockwise
    vel = -sqrt(2.0 * -pos_remaining * accel);

    if (vel > vel_1)  // Check if we actually need to accelerate in  clockwise direction
      {

      if (vel_1 == 0)  
        vel = sqrt(2.0 * accel_x_dpos);
      else
        vel = vel_1 + abs(accel_x_dpos/ vel_1);
      if (vel > vel_max)
        vel = vel_max;
      }
    else if (vel < vel_1)
    {
    // Need to accelerate in  counter clockwise direction
    if (vel_1 == 0)
      vel = -sqrt(2.0 * accel_x_dpos);
    else
      vel = vel_1 - abs(accel_x_dpos/ vel_1);
    if (vel < -vel_max)
      vel = -vel_max;
    }
  //  SerialUSB.println(vel);
  
 
  dt = abs(dpos_x_1000000 / vel);
  
    while(t < t_1 + dt) {           //wait calculated dt 
    t = micros();
    }
  
  if (vel > 0)  pos += dpos;        //update setpoint
  else if (vel < 0) pos -= dpos;
  r= r0 + pos;
  
  //SerialUSB.print(micros()-start);
  //SerialUSB.print(" , ");
  
  t_1 = t;  
  vel_1 = vel;
  pos_remaining = pos_final-pos;
  
  }
  r = r0 +pos_final;
  //SerialUSB.print(micros()-start);
}

void moveAbs(float pos_final,int vel_max, int accel){
  
   //Use this function for slow absolute movements in closed loop position mode
   //
   // This function creates a "trapezoidal speed" trajectory (constant accel, and max speed, constant decel);
   // It works pretty well, but it may not be perfect
   // 
   // pos_final is the desired position in degrees
   // vel_max is the max velocity in degrees/second
   // accel is the max accel in degrees/second^2
   //
   //Note that the actual max velocity is limited by the execution speed of all the math below.
   //Adjusting dpos (delta position, or step size) allows you to trade higher speeds for smoother motion
   //Max speed with dpos = 0.225 degrees is about 180 deg/sec
   //Max speed with dpos = 0.45 degrees is about 360 deg/sec
  
  float pos = r;
  float dpos = 0.225*2.0*PI/360.0;  // "step size" in radians, smaller is smoother, but will limit max speed, keep below stepper step angle
  float vel = 0;      // 
  float vel_1 =0;
 // int start = micros(); //for debugging

  float accel_x_dpos = accel*dpos;  // pre calculate
  float dpos_x_1000000 = dpos*1000000.0; // pre calculate

  float pos_remaining = pos_final-pos;
  unsigned long dt =0; 
  unsigned long t = micros();
  unsigned long t_1 = t;


  // Assume we're decelerating and calculate speed along deceleration profile
  
  while (abs(pos_remaining) >(dpos/2)){  //(may not actually reach exactly so leave some margin
  
    if (pos_remaining > 0)        // clockwise
    vel = sqrt(2.0 * pos_remaining * accel);
    else                      // counter clockwise
    vel = -sqrt(2.0 * -pos_remaining * accel);

    if (vel > vel_1)  // Check if we actually need to accelerate in  clockwise direction
      {

      if (vel_1 == 0)  
        vel = sqrt(2.0 * accel_x_dpos);
      else
        vel = vel_1 + abs(accel_x_dpos/ vel_1);
      if (vel > vel_max)
        vel = vel_max;
      }
    else if (vel < vel_1)
    {
    // Need to accelerate in  counter clockwise direction
    if (vel_1 == 0)
      vel = -sqrt(2.0 * accel_x_dpos);
    else
      vel = vel_1 - abs(accel_x_dpos/ vel_1);
    if (vel < -vel_max)
      vel = -vel_max;
    }
  //  SerialUSB.println(vel);
  
 
  dt = abs(dpos_x_1000000 / vel);
  
    while(t < t_1 + dt) {           //wait calculated dt 
    t = micros();
    }
  
  if (vel > 0)  pos += dpos;        //update setpoint
  else if (vel < 0) pos -= dpos;
  r= pos;
  
  //SerialUSB.print(micros()-start);    //for debugging
  //SerialUSB.print(" , ");
  
  t_1 = t;  
  vel_1 = vel;
  pos_remaining = pos_final-pos;
  
  }
  r = pos_final;
  //SerialUSB.print(micros()-start);
}
