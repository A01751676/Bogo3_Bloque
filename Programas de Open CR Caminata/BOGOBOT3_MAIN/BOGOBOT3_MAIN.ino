/*  BOGOBOT 3 PROGRAM
    Written by: Ana Patricia Islas Mainou
 */
 
#include "DXL_MX_ALL_MOTORS.h"
#include "DXL_MX.h"
#include "B3_POS.h"
#include "B3_FK_IK.h"
#include "B3_ROBOT_MOVE.h"
#include "ZMP.h"
#include <DynamixelSDK.h>

// Define dynamixel IDs
double IDs[18] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
double torque = 1;
void setup() {

  // Initialize PortHandler instance
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  
  // Initialize PacketHandler instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  // Initialize GroupSyncWrite instances
  dynamixel::GroupSyncWrite groupSyncWritePos(portHandler, packetHandler, 30, LEN_MX_GOAL_POSITION); // for position
  dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, 32, LEN_MX_GOAL_POSITION); // for velocity
  
  // Initialize BulkRead instances
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler); // present pos
  
  Serial.begin(115200);
  
  // Start Communication and Enable Torque
  if (startCom(portHandler,packetHandler,BAUDRATE)) {
    Serial.println("Start...");
  } else {
    return;
  }
  setTorqueAll(portHandler,packetHandler,IDs, torque);

  // MAIN ----------------------------------------------------------------------------------  
  //                 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
  double step[20] = {1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3};

  //Serial.println("pose T");
  //moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,configPos_QVals);
  //delay(20000);
  //moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p0);
  //moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p1);
  //println("pose parado");
  //moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,standPos_QVals);

  //Serial.println("pose precaminar");
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

  //Serial.println("caminar");
  for (int i=0; i<4; i++){
  // este ya jala NO MOVER -------------------------------------------------------------------
  // caminata al frente
  //walkSeq(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,step,3.5,0.4,6.5,7.8);
  // -----------------------------------------------------------------------------------------

  // caminata atras 
  walkSeq(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,step,6.2,0.3,-11.5,9.8);

  }

  //walkSideSeq(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,step,5.5,0.4,4,8.2);
  moveRobot_byPose(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,p2);

  // Stop Communication and Disable Torque
  moveRobot_byQVals(portHandler,packetHandler,groupSyncWritePos,groupSyncWriteVel,IDs,sitPos_QVals);
  delay(500);
  torque = 0;
  setTorqueAll(portHandler,packetHandler,IDs, torque);
  delay(500);
  stopCom(portHandler,packetHandler);

}


void loop() {}
