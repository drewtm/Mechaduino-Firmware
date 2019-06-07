#include <FlashStorage.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"
#include "calibrations.h"

static FlashClass flash;
static const unsigned page_size = 256; // actual size is 64?
static unsigned page_count;
static const unsigned floats_per_page = page_size / sizeof(float);
static float page[floats_per_page];
static const void * page_ptr;

static void write_page()
{
  if (0 == (0xFFF & (uintptr_t) page_ptr))
  {
    SerialUSB.println();
    SerialUSB.print("0x");
    SerialUSB.print((uintptr_t) page_ptr, HEX);
  } else {
    SerialUSB.print(".");
  }

  flash.erase((const void*) page_ptr, sizeof(page));
  flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
}

static void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != floats_per_page)
    return;

  // we've filled an entire page, write it to the flash
  write_page();

  // reset our counters and increment our flash page
  page_ptr += sizeof(page);
  page_count = 0;
  memset(page, 0, sizeof(page));
}

//unwrap small encoder count differences to go below 0 and above ENCODERCOUNTS, for averaging purposes
int encoderWrap(int thisReading, int lastReading){ 
  if ((thisReading-lastReading)<(-(ENCODERCOUNTS/2))){
    return(thisReading+ENCODERCOUNTS);
  }
  else if ((thisReading-lastReading)>((ENCODERCOUNTS/2))){
    return(thisReading-ENCODERCOUNTS);
  }
  else return(thisReading);
}

void calibrate() {   /// this is the calibration routine
  
  bool actuallyFlash = true;         //enable this to actually write the calibration to flash memory
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  int avg = 8*2;                      //how many readings to average. must be an even number.
  int delaytime = 100;                 //ms to wait for a step movement

  int iStart = 0;                     //encoder zero position index
  int jStart = 0;
  int stepNo = 0;
  
  int fullStepReading[spr];
  int phaseDiff[4] = {0, 0, 0, 0};
  int PHord[4] = {0, 1, 2, 3};
  
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;

  SerialUSB.println("Beginning calibration.");
  SerialUSB.println("The stepper will move a bit, then spin at a moderate speed to the encoder zero postion, then slowly one revolution forward and one backward to calibrate.");
  SerialUSB.println();
  
  disableTCInterrupts();      //make sure to disable closed loop
  fullStepReading[0] = readEncoder(); //discard an encoder read because the first reading after powerup is sometimes wrong
  
  //align rotor to current output phase in case this is the first thing after powerup
  dir = true;
  oneStep();
  oneStep();
  oneStep();
  dir = false;
  oneStep();
  oneStep();
  oneStep();
  dir = true;
  delay(delaytime);
  
  fullStepReading[0] = readEncoder(); //see where we're at
  //make sure we're not going into this within 4 steps of the rollover point of the encoder
  if( ( (ENCODERCOUNTS-fullStepReading[0]) < (ENCODERCOUNTS/spr*4) ) || (fullStepReading[0] < (ENCODERCOUNTS/spr*4) ) ) {
    SerialUSB.println("near rollover! moving away to check encoder orientation...");
    SerialUSB.println();
    for (int x=0; x<12; x++){
      oneStep();
      delay(delaytime/8);
    }
  }
  
  fullStepReading[0] = readEncoder();
  oneStep();
  delay(delaytime);

  if ((readEncoder() - fullStepReading[0]) < 0) {    //if encoder is counting backward for the dir=true case
    SerialUSB.println("Switching direction. Motor was wired backward from the encoder direction. It's fine for now.");
    SerialUSB.println();
    SerialUSB.println("However, you need to manually change the sign of 'int directionSwap' in Parameters.cpp");
    SerialUSB.println("    when you also copy over the calibration table, before you re-upload the firmware.");
    SerialUSB.println("If you don't, the firmware will malfunction next time you power cycle.");
    SerialUSB.println();
    directionSwap*=(-1);
    oneStep();
    oneStep();
  }

  //quickly find the full step position that corresponds to the lowest encoder reading
  lastencoderReading = readEncoder();
  do{
    oneStep();
    delay(delaytime/8);
  } while(readEncoder() > lastencoderReading);
  //if(directionSwap<0) oneStep(); //switching the commutation direction also changes the offset of the calibration table by one step

  SerialUSB.println("  0         PI         2PI");
  SerialUSB.print("   ");
  for (int x = 0; x < spr; x++) {     //step through all full step positions forward, recording their encoder readings

    fullStepReading[x] = 0;
    delay(delaytime);                     //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
        
    for (int reading = 0; reading < avg/2; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();
      currentencoderReading = encoderWrap(currentencoderReading, lastencoderReading);
      fullStepReading[x] += currentencoderReading;       //start accumulating many readings in this array
      delay(2);                                          //wait a little bit between readings
      lastencoderReading = currentencoderReading;
    }
    if(actuallyFlash){
      if(x%10==5) SerialUSB.print(">");
    }
    else{
      SerialUSB.print(x);
      SerialUSB.print(" , ");
      SerialUSB.println(fullStepReading[x]/(avg/2), DEC);
    }
    oneStep();
  }
  delay(delaytime);
  SerialUSB.println();
  SerialUSB.println("2PI         PI         0");
  SerialUSB.print("   ");
  //scoot forward to get a full commutation cycle of backward motion hysteresis before taking readings
  for(int k = 1; k<=4; k++) {oneStep(); delay(delaytime);}
  //now run backward to just past the wraparound, to the step with the highest encoder count
  dir=false;
  for(int k = 4; k>=0; k--) {oneStep(); delay(delaytime);}
  
  //now step through all full step positions backward, recording their encoder readings
  for (int x = spr-1; x >=0; x--) {           //note the backward iterator, used to make x the same for each step as it was in the forward direction
    delay(delaytime);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
    for (int reading = 0; reading < avg/2; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();
      currentencoderReading = encoderWrap(currentencoderReading, lastencoderReading);
      fullStepReading[x] += currentencoderReading;
      delay(2);
      lastencoderReading = currentencoderReading;
    }
    
    fullStepReading[x] = fullStepReading[x] / avg;
    
    if (fullStepReading[x] > ENCODERCOUNTS)
      fullStepReading[x] -= ENCODERCOUNTS;
    else if (fullStepReading[x] < 0)
      fullStepReading[x] += ENCODERCOUNTS;
    
    if(actuallyFlash){
      if(x%10==5) SerialUSB.print(">");
    }
    else{
      SerialUSB.print(x);
      SerialUSB.print(" , ");
      SerialUSB.println(fullStepReading[x], DEC);
    }
    oneStep();
  }

  dir=true;
  oneStep(); //go back around the wrap to the step with minimum encoder count;

  //for each of the four phase commutation modes, find the average distance from the previous one.
  //this is used to identify discrepancy between the two phases.
  for (int x=1; x<spr-1; x++) phaseDiff[x%4] += ( 2*fullStepReading[x] - fullStepReading[x-1] - fullStepReading[x+1] );
  phaseDiff[0] += ( 2*fullStepReading[0] - (fullStepReading[spr-1]-ENCODERCOUNTS) - fullStepReading[1]);  //wrap the first difference to the other end of the array
  phaseDiff[3] += ( 2*fullStepReading[spr-1] - fullStepReading[spr-2] - (fullStepReading[0]+ENCODERCOUNTS) ); //wrap the last difference to the other end of the array

  for(int x=0; x<4; x++) {
    phaseDiff[x] = round( float(phaseDiff[x]) / float(spr/4) / 2.0 );
  }
  
  //SerialUSB.println();
  for (int x=spr-1; x>=0; x--) {
    fullStepReading[x] -= phaseDiff[x%4];
    //SerialUSB.print(fullStepReading[x]);
    //SerialUSB.print(", ");
  }
  SerialUSB.println();
  SerialUSB.println();
  
  /*//sort the four phases from highest to lowest offset
  for(int x=0; x<3; x++){
    for(int xx=x+1; xx<4; xx++){
      if(phaseDiff[PHord[x]] < phaseDiff[PHord[xx]]){
        int temp = PHord[x];
        PHord[x] = PHord[xx];
        PHord[xx] = temp;
      }
    }
  }*/

 // SerialUSB.println(" ");
 // SerialUSB.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
 // SerialUSB.println(" ");
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReading[mod((i + 1), spr)] - fullStepReading[mod((i), spr)];
    if (ticks < -15000) {
      ticks += ENCODERCOUNTS;

    }
    else if (ticks > 15000) {
      ticks -= ENCODERCOUNTS;
    }
   // SerialUSB.println(ticks);

    if (ticks > 1) {                                    //note starting point with iStart,jStart
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReading[i] + j, ENCODERCOUNTS));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {                                    //note starting point with iStart,jStart
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReading[spr - 1 - i] + j, ENCODERCOUNTS));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

  }

  // The code below generates the lookup table by intepolating between
  // full steps and mapping each encoder count to a calibrated angle
  // The lookup table is too big to store in volatile memory,
  // so we must generate and store it into the flash on the fly

  // begin the write to the calibration table
  page_count = 0;
  page_ptr = (const uint8_t*) lookup;
  if(actuallyFlash) SerialUSB.print("Writing to flash 0x");
  if(actuallyFlash) SerialUSB.print((uintptr_t) page_ptr, HEX);
  if(actuallyFlash) SerialUSB.print(" page size PSZ=");
  if(actuallyFlash) SerialUSB.print(NVMCTRL->PARAM.bit.PSZ);

  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReading[mod((i + 1), spr)] - fullStepReading[mod((i), spr)];

    if (ticks < -15000) {           //check if current interval wraps over encoder's zero positon
      ticks += ENCODERCOUNTS;
    }
    else if (ticks > 15000) {
      ticks -= ENCODERCOUNTS;
    }
    //Here we print an interpolated angle corresponding to each encoder count (in order)
    if (ticks > 1) {              //if encoder counts were increasing during cal routine...

      if (i == iStart) { //this is an edge case
        for (int j = jStart; j < ticks; j++) {
          if(actuallyFlash) store_lookup(mod(10000.0 * ((aps * i) + ((aps * j ) / float(ticks))), round(2.0*PI*10000.0))/10000.0);
        }
      }

      else if (i == (iStart + spr)) { //this is an edge case
        for (int j = 0; j < jStart; j++) {
          if(actuallyFlash) store_lookup(mod(10000 * ((aps * i) + ((aps * j ) / float(ticks))), round(2.0*PI*10000.0))/10000.0);
        }
      }
      else {                        //this is the general case
        for (int j = 0; j < ticks; j++) {
          if(actuallyFlash) store_lookup(mod(10000 * ((aps * i) + ((aps * j ) / float(ticks))), round(2.0*PI*10000.0))/10000.0);
        }
      }
    }

    else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
      if (i == iStart) {
        for (int j = - ticks; j > (jStart); j--) {
          if(actuallyFlash) store_lookup(mod(10000.0 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), round(2.0*PI*10000.0))/10000.0);
        }
      }
      else if (i == iStart + spr) {
        for (int j = jStart; j > 0; j--) {
          if(actuallyFlash) store_lookup(mod(10000.0 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), round(2.0*PI*10000.0))/10000.0);
        }
      }
      else {
        for (int j = - ticks; j > 0; j--) {
          if(actuallyFlash) store_lookup(mod(int(10000.0 * (aps * (i) + (aps * ((ticks + j)) / float(ticks)))), round(2.0*PI*10000.0))/10000.0);
        }
      }

    }

  }

  if ((page_count!=0) && (actuallyFlash)) write_page();
  SerialUSB.println(" ");
  if(actuallyFlash){
    SerialUSB.println(" ");
    SerialUSB.println("Calibration complete!");
    SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
    SerialUSB.println(" ");
    SerialUSB.println(" ");
  }
}

void calibrationQuery() {         //print the lookup table
  SerialUSB.println("");
  SerialUSB.println("//This is the encoder lookup table (created by calibration routine)");
  SerialUSB.println("");
  
  SerialUSB.println("const float __attribute__((__aligned__(256))) lookup[ENCODERCOUNTS] = {");
  for (int i = 0; i < ENCODERCOUNTS; i++) {
    SerialUSB.print(lookup[i],4);
    SerialUSB.print(", ");
  }
  SerialUSB.println("");
  SerialUSB.println("};");
}


void antiCoggingCal() {       //This is still under development...  The idea is that we can calibrate out the stepper motor's detent torque by measuring the torque required to hold all possible positions.
  SerialUSB.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
  mode = 'x';
  ITerm = 0;
  output(0,(int)(uMAX/2.0));  //snap the stepper to the nearest beginning of a commutation cycle
  delay(500);
  int startCnt = readEncoder();
  float startPos = lookup[startCnt];
  r = startPos;
  int Npoints = 250;
  float travel = aps*16.0;
  float interval = travel/float(Npoints);
  enableTCInterrupts();
  delay(10000);

  for (int i = 0; i < Npoints; i++) {
    int avg_ct = 10;
    float u_avg = 0.0;
    float y_avg = 0.0;
    r = startPos+(float(i)*interval);
    delay(20);
    for(int j=0; j<avg_ct; j++){
      u_avg += u;
      y_avg += y;
      delay(1);
    }
    SerialUSB.print(y_avg/avg_ct*10.0, DEC);
    SerialUSB.print(" , ");
    SerialUSB.println(u_avg/avg_ct, DEC);
  }
  SerialUSB.println(" -----------------REVERSE!----------------");

  for (int i = Npoints; i > 0; i--) {
    int avg_ct = 10;
    float u_avg = 0.0;
    float y_avg = 0.0;
    r = startPos+(float(i)*interval);
    delay(20);
    for(int j=0; j<avg_ct; j++){
      u_avg += u;
      y_avg += y;
      delay(1);
    }
    SerialUSB.print(y_avg/avg_ct*10.0, DEC);
    SerialUSB.print(" , ");
    SerialUSB.println(u_avg/avg_ct, DEC);
  }
  SerialUSB.println(" -----------------DONE!----------------");
  disableTCInterrupts();
  output(0,0);
}


void sineGen() {
  int temp;
     SerialUSB.println("");
     SerialUSB.println("The sineGen() function in Utils.cpp generates a sinusoidal commutation table.");
     SerialUSB.println("You can experiment with different commutation profiles by modifying this table.");
     SerialUSB.println("The below table should be copied into sine_1 in Parameters.cpp.");   
     SerialUSB.println("");
     delay(3000);
     SerialUSB.println("Printing sine look up table:...");
     SerialUSB.println("");
  for (int x = 0; x < sineTableSize; x++) {
    temp = round(1024.0 * sin(2.0*PI*float(x)/float(sineTableSize)));
   SerialUSB.print(temp);
   SerialUSB.print(", ");  
  }
}
