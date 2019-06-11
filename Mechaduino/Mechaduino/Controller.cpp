//Contains TC5 Controller definition
//The main control loop is executed by the TC5 timer interrupt:

#include <SPI.h>

#include "State.h"
#include "Utils.h"
#include "Parameters.h"

void TC5_Handler() {                                   // gets called with FPID frequency
  static int print_counter = 0;                        //this is used by step response
  int encoderRaw;
  static int oldEncoderRaw = -1;
  bool stationary = true;
  int phaseSign = 1;

  //for mass simulation
  float inertia = 1.4;
  float damping = 0.08;
  static float virtualVel = 0.0;
  float Ksim = 26.0;

  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {    // A counter overflow caused the interrupt
     
    TEST1_HIGH();  //digitalWrite(3, HIGH);       //Fast Write to Digital 3 for debugging

    encoderRaw = readEncoder();
    if(encoderRaw==oldEncoderRaw){
      oldSlowCnt++; //avoid overflow during very long periods of no motion
      stationary = true;
    }
    else{
      stationary = false;
      oldEncoderRaw = encoderRaw;
      oldSlowCnt = 0;
    }
    y = lookup[encoderRaw];                    //read encoder and lookup corrected angle in calibration lookup table
   
    if ((y - y_1) < -PI) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 2PI to 0 or from 0 to 2PI?)
    else if ((y - y_1) > PI) wrap_count -= 1;

    yw = (y + (2.0*PI * wrap_count));              //yw is the wrapped angle (can exceed one revolution)

    if (mode == 'h') {                            //choose control algorithm based on mode
      hybridControl();                            // hybrid control is still under development...
    }
    else {
      switch (mode) {
        case 'x':         // position control
          //=== error in radians
          e = (yw - r);          
          //=== derivative of error, rad/sec. calculated in a different way at very slow speeds.
          if(stationary) de = pLPF.filterIn((e-oldSlow)*Fs/float(oldSlowCnt));
          else if(oldSlowCnt!=0) { //if this is the first encoder count after a stationary period of time
            de = pLPF.filterIn((e-oldSlow)*Fs/float(oldSlowCnt));
            oldSlow = e;
          }
          else { //if the motor is moving fast enough to get encoder counts every loop
            de = pLPF.filterIn((e-e_1)*Fs);
            oldSlow = e;
          }
          //=== integral of error, rad*sec
          ITerm += e*Ts;
          //Integral wind up limit
          if (ITerm > pAWi) ITerm = pAWi;
          else if (ITerm < -pAWi) ITerm = -pAWi;          
          //=== apply gains and calculate motor command
          u = pKp*e + pKd*de + pKi*ITerm;
          u *= (-1.0);
          break;
            
        case 'v':         // velocity controller
          //=== velocity, rad/sec. calculated in a different way at very slow speeds.
          if(stationary) v = vLPF.filterIn((yw-oldSlow)*Fs/float(oldSlowCnt));
          else if(oldSlowCnt!=0) { //if this is the first encoder count after a static period of time
            v = vLPF.filterIn((yw-oldSlow)*Fs/float(oldSlowCnt));
            oldSlow = yw;
          }
          else { //if the motor is moving fast enough to get encoder counts every loop
            v = vLPF.filterIn((yw-yw_1)*Fs);
            oldSlow = yw;
          }
          //=== error, rad/s
          e = (v - r);
          //=== derivative of error, (rad/sec)/sec
          de = aLPF.filterIn((e-e_1)*Fs);
          //=== integral of error, (rad/s)*sec
          ITerm += e*Ts;
          //Integral wind up limit
          if (ITerm > vAWi) ITerm = vAWi;
          else if (ITerm < -vAWi) ITerm = -vAWi;
          //=== apply gains
          u = vKp*e + vKd*de + vKi*ITerm;
          u *= (-1.0);
          break;
          
        case 't':         // torque control
          u = tK * r ;
          break;

        case 's':         // simulation
          //=== velocity, rad/sec. calculated in a different way at very slow speeds.
          if(stationary) v = vLPF.filterIn((yw-oldSlow)*Fs/float(oldSlowCnt));
          else if(oldSlowCnt!=0) { //if this is the first encoder count after a static period of time
            v = vLPF.filterIn((yw-oldSlow)*Fs/float(oldSlowCnt));
            oldSlow = yw;
          }
          else { //if the motor is moving fast enough to get encoder counts every loop
            v = vLPF.filterIn((yw-yw_1)*Fs);
            oldSlow = yw;
          }
          e = (v-virtualVel);               //difference between reality and simulation
          virtualVel *= (1.0-damping*Ts);   //
          virtualVel += (e/inertia)*Ts;     //virtual acceleration
          u = -Ksim*e;                      //try to make the output follow the simulation
          break;
        default:
          u = 0;
          break;
      }

    if(u>uMAX) u = uMAX;          //constrain control signal to max allowable value
    else if(u<-uMAX) u = -uMAX;   // "saturation" limit
      
      if (abs(e) < aps/2.0) ledPin_HIGH();    // turn on LED if error is less than 0.1
      else ledPin_LOW();                  //digitalWrite(ledPin, LOW);
      
      output(y, round(u), PA);    // update phase currents
    }
    
    y_1 = y;
    v_1 = v;      //copy current values to previous values for next control cycle
    e_2 = e_1;    //these past values can be useful for more complex controllers/filters.  Uncomment as necessary    
    e_1 = e;
    u_2 = u_1;
    u_1 = u;
    yw_1 = yw;
    if (print_yw==true){       //for debugging
      print_counter += 1;  
      if (print_counter >= 100){    // print position every so often (every time is too much data for plotter and may slow down control loop
        SerialUSB.print(virtualVel);
        SerialUSB.print(" , ");
        SerialUSB.println(u);
        print_counter = 0;
      }
    }
    
    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    TEST1_LOW();            //for testing the control loop timing
  }

}
