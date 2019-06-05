//Contains the declarations for the serial menu functions used by the firmware

#ifndef __SERUT_H__
#define __SERUT_H__
  
  void serialCheck();               //checks serial port for commands.  Must include this in loop() for serial interface to work
  
  void parameterQuery();            //Prints current parameters
  
  void print_angle();               //for debigging purposes in open loop mode:  prints [step number] , [encoder reading]
      
  void readEncoderDiagnostics();    //check encoder diagnostics registers
    
  void parameterEditmain();         //parameter editing menu
  
  void parameterEditp();            //parameter editing menu
  
  void parameterEditv();            //parameter editing menu
  
  void parameterEdito();            //parameter editing menu
  
  void serialMenu();                //main menu

#endif
