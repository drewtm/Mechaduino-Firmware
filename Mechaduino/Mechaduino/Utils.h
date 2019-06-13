//Contains the declarations for the functions used by the firmware

#ifndef __UTILS_H__
#define __UTIL_H__

	void setupPins();                 // initializes pins
	
	void setupSPI();                  //initializes SPI

  void configureStepDir();          //configure step/dir interface
  
  void configureEnablePin();        //configure enable pin 
		
	void stepInterrupt();             //step interrupt handler

  void dirInterrupt();              //dir interrupt handler

  void enableInterrupt();           //enable pin interrupt handler

	void output(float theta, int effort, int phaseShift = 0);	  //calculates phase currents (commutation) and outputs to Vref pins

  float read_angle();               //get an average from a few readings from the encoder
	
	void oneStep(void);               //take one step
		
	int readEncoder();                //read raw encoder position
	
	void receiveEvent(int howMany);   //for i2c interface...
	
	int mod(int xMod, int mMod);      //modulo, handles negative values properly

//  float mod(float xMod, float mMod);  //floating point modulo with negative values
	
	void setupTCInterrupts();         //configures control loop interrupt
	
	void enableTCInterrupts();        //enables control loop interrupt.  Use this to enable "closed-loop" modes
	
	void disableTCInterrupts();       //disables control loop interrupt.  Use this to diable "closed-loop" mode

  void hybridControl();             //open loop stepping, but corrects for missed steps.  under development

  void stepResponse();              //generates position mode step response in Serial Plotter

  void moveRel(float pos_final,int vel_max, int accel);     // Generates trapezoidal motion profile for closed loop position mode
  
  void moveAbs(float pos_final,int vel_max, int accel);     // Generates trapezoidal motion profile for closed loop position mode
  
#endif
