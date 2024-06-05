#ifndef DXL_MX_CONV
#define DXL_MX_CONV

/*  CONVERSIONS NEEDED FOR DYNAMIXEL MOTORS MX SERIES
 *   
 *  Available functions:
 * 
 *  void deg2bits().....................Convertion fron degrees to bits.............................................OK
 *  void rad2bits().....................Convertion fron radians to bits.............................................OK
 *  void bits2deg().....................Conversion from bits to degrees.............................................OK
 *  void bits2rad().....................Conversion from radians to bits.............................................OK
 *  int calcSpeedBits().................Calculates moving speed in bits form radians................................OK
 *  void calcMotorsSpeed()..............Calculates moving speed in bits form radians for all motors.................OK
 *  
 */
#include "MATH_AUX.h"

double deg2bits(double deg, double offset){
  return ceil(min_(((deg/0.0879) + offset),4095));
}

double rad2bits(double rad, double offset){
  return ceil(min_((rad*(180.0/(0.0879 * PI)) + offset),4095));
}

double bits2deg(double  bits, double offset){
  return (bits-offset)*0.0879;
}

double  bits2rad(double bits, double offset){
  return (bits-offset)*((0.0879)*(PI/180.0));
}

double calcSpeedBits(double x0,double xf, double t){
  double rad_s = abs((xf-x0)/t);
  double b = rad_s*(1/(0.1047 * 0.114));
  //Serial.print(t);
  double  a = min_(b,4095);
  return ceil(max_(1,a));
}

void calcMotorsSpeed(double q0[20], double qf[20], double qVel[18]){
  for (int i=0; i<18; i++){
    double p0 = bits2rad(q0[i], configPos_QVals[i]);
    double pf = bits2rad(qf[i], configPos_QVals[i]);
    qVel[i] = calcSpeedBits(p0,pf,qf[18]);
    //Serial.print(qVel[i]);
    //Serial.print(" ");
    }
    //Serial.println("");
}


#endif
