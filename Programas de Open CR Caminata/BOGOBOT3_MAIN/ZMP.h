#ifndef ZMP
#define ZMP

#include "MATH_AUX.h"
#include "B3_POS.h"
#include "B3_ROBOT_MOVE.h"

double g = 980; double h = 1; double s = 20; double hIK = 31;
double dt = 0.1;

// Tray Z
double pz(double t, double T, double radio){
  double z = -hIK +radio*(2/T)*sqrt(t*(T-t));
  Serial.println(z);
  return z;
}

// ZMP y
double py(double t, double tf, double Yzmp){
  double k1 = (-Yzmp)/(1+exp(sqrt(g/h)*tf));
  double k2 = k1*exp(sqrt(g/h)*tf);
  return k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t) + Yzmp;
}

// ZMP X

double px(double t, double k1, double k2){
  double a = k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t);
  //Serial.print(a);
  return a;
}

double front(double tf, double Xzmp){
  double k1 = (-Xzmp)/(1-exp(sqrt(g/h)*tf));
  double k2 = -k1*exp(sqrt(g/h)*tf);
  return k1, k2;
}

double openFront(double tf, double Xzmp){
  double k1 = (-Xzmp*exp(sqrt(g/h)*tf)) / ((1-exp(sqrt(g/h)*tf))*(1+exp(sqrt(g/h)*tf)));
  double k2 = -k1;
  return k1, k2;
}

double closeFront(double tf, double Xzmp){
  double k1 = (-Xzmp) / ((1-exp(sqrt(g/h)*tf))*(1+exp(sqrt(g/h)*tf)));
  double k2 = -k1*exp(2*sqrt(g/h)*tf);
  return k1, k2;
}

// ZMP Trayectory 
double zmpX_FwdCase(double step, double tf, double Xzmp){
  if (step == 1){ // open step
    return openFront(tf, Xzmp);}
  
  else if (step == 3){ // close step
    return closeFront(tf, Xzmp);}

  else{
    return front(tf, Xzmp);}
}

double zmpY_foot(int i, double yzmp){
  if (i%2 == 0){return -yzmp;}
  else{return yzmp;}
}

double walkSeq(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double step[20], double radio, double tf, double Xzmp, double yzmp){
  for (int i=0; i<s; i++){
    double t = 0;
    double stop = t +tf;
    Serial.println("------------- inicio ciclo -----------");
    while (t < stop){

      double k1, k2 = zmpX_FwdCase(step[i], tf, Xzmp);
      double Yzmp = zmpY_foot(i, yzmp);
      // pie fijo ---------------------------------
      // puntos del pie a la cadera
      double yFijo = py(t, tf, Yzmp);
      double xFijo = px(t, k1, k2);

      // puntos de la cadera al pie para pie fijo
      xFijo = -xFijo;
      yFijo = -yFijo;
      double zFijo = -hIK;

      // pie movil -------------------------------
      double xMovil = 0;
      double yMovil = yFijo;
      double zMovil = pz(t, stop, radio);

      if (Yzmp > 0){// si yzmp es positivo usar pierna izq armar pose acorde
        // mover el centro de masa con la pierna izquierda 
        walk_TaskS[0] = xFijo; // x left leg
        walk_TaskS[1] = yFijo; // y left leg
        walk_TaskS[2] = zFijo; // z left leg
        // dar paso con la pierna derecha
        walk_TaskS[4] = xMovil; // x right leg
        walk_TaskS[5] = yMovil; // y right leg
        walk_TaskS[6] = zMovil; // z right leg 
      }
      else{
        // mover el centro de masa con la pierna derecha 
        walk_TaskS[4] = xFijo; // x right leg
        walk_TaskS[5] = yFijo; // y right leg
        walk_TaskS[6] = zFijo; // z right leg
        // dar paso con la pierna izquierda
        walk_TaskS[0] = xMovil; // x left leg
        walk_TaskS[1] = yMovil; // y left leg
        walk_TaskS[2] = zMovil; // z leftt leg  

      }
      walk_TaskS[14] = dt;
      walk_TaskS[15] = 0;

      moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,walk_TaskS);

      /*
      Serial.print(xFijo);
      Serial.print(" ");
      Serial.print(yFijo);
      Serial.print(" ");
      Serial.print(zFijo);
      Serial.print(" ");
      Serial.print(xMovil);
      Serial.print(" ");
      Serial.print(yMovil);
      Serial.print(" ");
      Serial.println(zMovil);*/
      t = t + dt;

    }
  }
}

// ----------------------------------------------------------------------------------------------------------------

double pySide(double t0, double tf, double yf, double Py){
  double Nstep = (tf-t0)/dt;
  double dy = (yf- 0)/Nstep;
  Py = Py + dy;
  return Py;
}

double walkSideSeq(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double step[20], double radio, double tf, double StepSize, double yzmp){

  for (int i=0; i<s; i++){
    double t = 0;
    double stop = t +tf;
    double yMovil = 0;
    Serial.println("------------- inicio ciclo -----------");
    while (t < stop){

      //double k1, k2 = zmpX_FwdCase(step[i], tf, Xzmp);
      double Yzmp = zmpY_foot(i, yzmp);
      // pie fijo ---------------------------------
      // puntos del pie a la cadera
      double yFijo = py(t, tf, Yzmp);
      double xFijo = 0;

      // puntos de la cadera al pie para pie fijo
      //xFijo = -xFijo;
      yFijo = -yFijo;
      double zFijo = -hIK;

      // pie movil -------------------------------
      double xMovil = 0;
      double st = 0;
      if (Yzmp < 0){
        st = 8;
        yMovil = yFijo +st;
      }
      else{
        st = 0;
        yMovil = yFijo +st;
      }
      //yMovil = yFijo +st; //+ pySide(0,tf,StepSize,yMovil);
      double zMovil = pz(t, stop, radio);

      if (Yzmp > 0){// si yzmp es positivo usar pierna izq armar pose acorde
        // mover el centro de masa con la pierna izquierda 
        walk_TaskS[0] = xFijo; // x left leg
        walk_TaskS[1] = yFijo; // y left leg
        walk_TaskS[2] = zFijo; // z left leg
        // dar paso con la pierna derecha
        walk_TaskS[4] = xMovil; // x right leg
        walk_TaskS[5] = yMovil; // y right leg
        walk_TaskS[6] = zMovil; // z right leg 
      }
      else{
        // mover el centro de masa con la pierna derecha 
        walk_TaskS[4] = xFijo; // x right leg
        walk_TaskS[5] = yFijo; // y right leg
        walk_TaskS[6] = zFijo; // z right leg
        // dar paso con la pierna izquierda
        walk_TaskS[0] = xMovil; // x left leg
        walk_TaskS[1] = yMovil; // y left leg
        walk_TaskS[2] = zMovil; // z leftt leg  

      }
      walk_TaskS[14] = dt;
      walk_TaskS[15] = 0;

      moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,walk_TaskS);

      
      Serial.print(xFijo);
      Serial.print(" ");
      Serial.print(yFijo);
      Serial.print(" ");
      Serial.print(zFijo);
      Serial.print(" ");
      Serial.print(xMovil);
      Serial.print(" ");
      Serial.print(yMovil);
      Serial.print(" ");
      Serial.println(zMovil);
      t = t + dt;

    }
  }
}

// ----------------------------------------------------------------------------------------------------------------

#endif