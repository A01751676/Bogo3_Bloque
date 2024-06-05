#ifndef B3_ROBOT_MOVE
#define B3_ROBOT_MOVE

#include "DXL_MX_ALL_MOTORS.h"
#include "B3_POS.h"
#include "B3_FK_IK.h"

double vel[18] = {};

/*  MOVES BOGOBOT 3 ONE POSTURE AT THE TIME
 *   
 *  Available functions:
 * 
 *  void moveRobot_byPose()........Moves robot with given coordinates (x,y,z).......................................OK
 *  void IK_RightLeg().............Moves robot using articular values in bits (q)...................................OK
 *  
 */
 
void moveRobot_byPose(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double p[16]){

  // desiered positions for:
  //                       X       Y       Z       alpha
  //    LEFT LEG  (LL):    p[0]    p[1]    p[2]    p[3]
  //    RIGHT LEG (RL):    p[4]    p[5]    p[6]    p[7]
  //    LEFT ARM  (LA):    p[8]    p[9]    p[10]
  //    RIGHT ARM (RA):    p[11]   p[12]   p[13]

  // timigs:  
  //    PLAYTIME :  p[14]
  //    DELAYTIME : p[15]

  // calculate q of the robot with IK
  double LL[4] = {p[0],p[1],p[2],p[3]};
  double RL[4] = {p[4],p[5],p[6],p[7]};
  double LA[3] = {p[8],p[9],p[10]};
  double RA[3] = {p[11],p[12],p[13]};

  IK_LeftLeg(LL, q);
  IK_RightLeg(RL, q);
  //IK_RightArm(RA,q);

  // stablish playtime and delay
  q[18] = p[14]; // playtime
  q[19] = p[15]; // delay

  // get the robot current position
  getPositionAll(portHandler,packetHandler, IDs, q0);

  // calculate and set desiered speed
  calcMotorsSpeed(q0, q, vel);
  setVelocitySync(groupSyncWriteVel, packetHandler, IDs,vel);

  // set the robot position
  setPositionSync(groupSyncWritePos, packetHandler,IDs, q);
  // stop aftermovement
  //delay(q[19]*1000);
}

void moveRobot_byQVals(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
               dynamixel::GroupSyncWrite groupSyncWritePos, dynamixel::GroupSyncWrite groupSyncWriteVel,
               double IDs[18], double q[20]){
  
  // get the robot current position
  getPositionAll(portHandler,packetHandler, IDs, q0);

  // calculate and set desiered speed
  calcMotorsSpeed(q0, q, vel);
  setVelocitySync(groupSyncWriteVel, packetHandler, IDs,vel);

  // set the robot position
  setPositionSync(groupSyncWritePos, packetHandler,IDs, q);

  // stop aftermovement
  //(q[19]*1000);
}
#endif