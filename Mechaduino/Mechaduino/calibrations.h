//Contains the declarations for the calibration functions used by the firmware

#ifndef __CALIB_H__
#define __CALIB_H__

  int encoderWrap(int thisReading, int lastReading);

  void calibrate();                 //calibration routine
  
  void calibrationQuery();          //print out the calibration table

  void antiCoggingCal();            //under development...

  void sineGen();                   //generates sinusoidal commutation table. you can experiment with other commutation profiles

#endif
