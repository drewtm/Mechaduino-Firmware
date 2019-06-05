//Contains the declaration of the state variables for the control loop  

//interrupt vars

volatile float y = 0.0;     // measured angle
volatile float yw = 0.0;    // "wrapped" angle (not limited to 0-2PI)
volatile float v = 0.0;     // estimated velocity  (velocity loop)
volatile float r = 0.0;     //setpoint
volatile float e = 0.0;     // e = r-y (error)
volatile float de = 0.0;    // derivative of error
volatile float ITerm;       // integrated error
//volatile float p = 0.0;   // proportional effort
//volatile float i = 0.0;   // integral effort
volatile float u = 0.0;     //real control effort (not abs)
volatile int U = 0;         //control effort (abs)

volatile float yw_1 = 0.0;  //previous wrapped angle
volatile float v_1 = 0.0;   //previous velocity
volatile float e_1 = 0.0;   //these past values can be useful for more complex controllers/filters     
volatile float u_1 = 0.0;   //value of u at previous time step, etc...
volatile float e_2 = 0.0;
volatile float u_2 = 0.0;

volatile float oldSlow = 0.0;  //for tracking a previous position at low speeds across many control loop passes

volatile long counter = 0;
volatile unsigned int oldSlowCnt = 0; //for tracking how many control loops have passed with no motion

volatile long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float y_1 = 0;
  
volatile long step_count = 0;  //For step/dir interrupt (closed loop)
int stepNumber = 0; // open loop step number (used by 's' and for cal routine)



char mode;
volatile bool dir = false;  

bool print_yw = true;      //for step response, under development...
