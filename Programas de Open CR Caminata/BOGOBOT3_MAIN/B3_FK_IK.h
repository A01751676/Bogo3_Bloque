#ifndef B3_FK_IK
#define B3_FK_IK

/*  INVERSE KINEMATICS FOR BOGOBOT 3
 *   
 *  Available functions:
 * 
 *  void IK_LeftLeg()..............Inverse Kinematics Left Leg......................................................OK
 *  void IK_RightLeg().............Inverse Kinematics Right Leg.....................................................OK
 *  
 */

#include "MATH_AUX.h"
#include "B3_POS.h"
#include "DXL_MX_CONV.h"

// index             0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17    18  19
// -------------------------------------------------------------------------------------------------------------------
// q structure       0001 0002 0003 0004 0005 0006 0007 0008 0009 0010 0011 0012 0013 0014 0015 0016 0017 0018  PT  D
// Pose structure    Px   Py   Pz   Alpha

// Dim structure     L1     L2     L3   L4
double L_DIM[4] =    {0.53, 0.465, 18.5,18.5};
double A_DIM[3] =    {0.33, 20,    19};

// Inverse Kinematics Left Leg
void IK_LeftLeg(double Pose[4], double q[20]){
// left leg IK       ---- ---- ---- ---- ---- ---- ---- 0008 ---- 0010 ---- 0012 ---- 0014 ---- 0016 ---- 0018  --  -
  double px = 0.0; double py = 0.0; double pz = 0.0;
  double L3 = L_DIM[2]; double L4 = L_DIM[3];

  // q8 -> pelvis YL
  double q8 = Pose[3];  double s8 = sin(q8); double c8 = cos(q8);
  px = Pose[0] * c8 - Pose[1] * s8;
  py = Pose[0] * s8 + Pose[1] * c8;
  pz = Pose[2];  

  /*
  Serial.print("px: ");
  Serial.print(px);
  Serial.print(" py: ");
  Serial.print(py);
  Serial.print(" pz: ");
  Serial.println(pz);*/

  // q14 -> knee L
  double c14 = (pow(px,2) + pow(py,2)+ pow(pz,2) - pow(L3,2) - pow(L4,2))/(2*L3*L4);
  c14 = max_(-1,min_(c14,1));
  double q14 = acos(c14); double s14 = sin(q14);

  double q10 = atan2_(-py,-pz); 
  double c10 = cos(q10); double s10 = sin(q10);

  double q12 = atan2_(-px, -pz*c10 -py*s10) - atan2_(L4*s14, L3 + L4*c14);
  double c12 = cos(q12); double s12 = sin(q12);

  double q16 = q14 + q12;
  double q18 = q10;

  // se realizan los cambios de sentido mecanicos 
  q10 = -1*q10;
  q18 = -1*q18;
  
  // convert to bits
  q[7] = rad2bits(q8, configPos_QVals[7]); //q8
  q[9] = rad2bits(q10, configPos_QVals[9]); //q10
  q[11] = rad2bits(q12, configPos_QVals[11]); //q12
  q[13] = rad2bits(q14, configPos_QVals[13]); //q14
  q[15] = rad2bits(q16, configPos_QVals[15]); //q16
  q[17] = rad2bits(q18, configPos_QVals[17]); //q18

  /*
  Serial.print("q8: ");
  Serial.print(q8);
  Serial.print(" q10: ");
  Serial.print(q10);
  Serial.print(" q12: ");
  Serial.print(q12);
  Serial.print(" q14: ");
  Serial.print(q14);
  Serial.print(" q16: ");
  Serial.print(q16);
  Serial.print(" q18: ");
  Serial.println(q18);*/
}

// Inverse Kinematics Right Leg
void IK_RightLeg(double Pose[4], double q[20]){
// right leg IK      ---- ---- ---- ---- ---- ---- 0007 ---- 0009 ---- 0011 ---- 0013 ---- 0015 ---- 0017 ----  --  -
  double px = 0.0; double py = 0.0; double pz = 0.0;
  double L3 = L_DIM[2]; double L4 = L_DIM[3];

  // q7 -> pelvis YR
  double q7 = Pose[3];  double s7 = sin(q7); double c7 = cos(q7);
  px = Pose[0] * c7 - Pose[1] * s7;
  py = Pose[0] * s7 + Pose[1] * c7;
  pz = Pose[2];  

  // q13 -> knee L
  double c13 = (pow(px,2) + pow(py,2)+ pow(pz,2) - pow(L3,2) - pow(L4,2))/(2*L3*L4);
  c13 = max_(-1,min_(c13,1));
  double q13 = - acos(c13); double s13 = sin(q13);

  double q9 = atan2_(-py,-pz); 
  double c9 = cos(q9); double s9 = sin(q9);

  double q11 = atan2_(px, -pz*c9 -py*s9) - atan2_(L4*s13, L3 + L4*c13);
  double c11 = cos(q11); double s11 = sin(q11);
  q11 = atan2_(s11,c11);

  double q15 = q13 + q11;
  double q17 = q9;

  // se realizan los cambios de sentido mecanicos 
  q9 = -1*q9;
  q17 = -1*q17;

  // convert to bits
  q[6] = rad2bits(q7, configPos_QVals[6]); //q7
  q[8] = rad2bits(q9, configPos_QVals[8]); //q9
  q[10] = rad2bits(q11, configPos_QVals[10]); //q11
  q[12] = rad2bits(q13, configPos_QVals[12]); //q13
  q[14] = rad2bits(q15, configPos_QVals[14]); //q15
  q[16] = rad2bits(q17, configPos_QVals[16]); //q17
  
  /*
  Serial.print("q7: ");
  Serial.print(q7);
  Serial.print(" q9: ");
  Serial.print(q9);
  Serial.print(" q11: ");
  Serial.print(q11);
  Serial.print(" q13: ");
  Serial.print(q13);
  Serial.print(" q15: ");
  Serial.print(q15);
  Serial.print(" q17: ");
  Serial.println(q17);*/
}

void IK_RightArm(double Pose[3], double q[20]){
  double px = Pose[0]; double py = Pose[1]; double pz = Pose[2];
  double L2 = A_DIM[1]; double L3 = A_DIM[2];

  // q5 
  double c5 = (pow(px,2) + pow(py,2)+ pow(pz,2) - pow(L2,2) - pow(L3,2))/(2*L2*L3);
  c5 = max_(-1,min_(c5,1));
  double q5 = acos(c5); double s5 = sin(q5); // revisar signo 

  // q3
  double A = -L2 -L3*c5;//-L3*s5;
  double B = -L3*s5;
  double C = py;
  double D = pow(A,2) + pow(B,2) - pow(C,2);
  if (D >= 0){D = 0;}
  double q3 = atan2_(B,A) + atan2_(sqrt(D),C); // revisar signo 
  double c3 = cos(q3); double s3 = sin(q3);
  q3 = atan2_(s3,c3); q3= -1*q3;

  // q1
  double s35 = sin(q3-q5);
  double q1 = atan2_((-px/(L2*s3 +L3*s35)) ,(pz/(L2*s3 +L3*s35)));

  q[0] = rad2bits(q1, configPos_QVals[0]); //q1
  q[2] = rad2bits(q3, configPos_QVals[2]); //q3
  q[4] = rad2bits(q5, configPos_QVals[4]); //q5

  Serial.println(q1);
  Serial.println(q3);
  Serial.println(q5);
}

void getValuesIK(double q[20]){
  for (int i=0; i<18; i++){
    Serial.print("[ID:");
    Serial.print(i+1);
    Serial.print("] calculated value: ");
    Serial.println(q[i]);
    }
}
#endif
