//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__

//interrupt vars

extern volatile float y;  // measured angle
extern volatile float yw; // wrapped angle (can be many rotations)
extern volatile float v;  // estimated velocity (velocity loop)
extern volatile float r;  // setpoint
extern volatile float e;  // e = yw-r (error)
extern volatile float de; // derivative of error
//extern volatile float p;  // proportional effort
extern volatile float ITerm;
extern volatile float u;  //real control effort (not abs)
//extern volatile float i;  // integral effort
extern volatile int U;  //control effort (abs)

extern volatile float yw_1;
extern volatile float v_1;
extern volatile float e_1;
extern volatile float u_1;
extern volatile float e_2;
extern volatile float u_2;

extern volatile float oldSlow;

extern volatile long counter;
extern volatile unsigned int oldSlowCnt;

extern volatile long wrap_count;
extern volatile float y_1;

extern volatile long step_count;  //For step/dir interrupt
extern int stepNumber; // step index for cal routine

extern char mode;
extern int dir;

extern bool print_yw;     //for step response, under development...
#endif
