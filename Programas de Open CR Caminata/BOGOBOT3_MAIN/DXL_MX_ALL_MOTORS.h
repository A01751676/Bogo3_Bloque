#ifndef DXL_MX_ALL_MOTORS
#define DXL_MX_ALL_MOTORS

/*   MANIPULATE ALL DYNAMIXEL MOTOS OF BOGOBOT 3
 *   
 *  Available functions:
 * 
 *  void setTorqueAll().................Sets torque value to all motors.............................................OK
 *  void setPositionAll()...............Sends position to all motors................................................OK
 *  void getPositionAll()...............Gets position from all motors...............................................OK
 *  void setPositionSync()..............Sync sends position to all motors...........................................OK
 *  void setVelocitySync()..............Sync sends speed to all motors..............................................OK
 *  void getPositionBulk()...!!!!!!
 */
 
#include "DXL_MX.h"
#include <DynamixelSDK.h>

// Data Byte Length for sync write
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2


void setTorqueAll(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,double IDs[18],uint8_t Torque_State){
    for (int i=0; i<18; i++){
    setTorque(portHandler,packetHandler, IDs[i], Torque_State);
    }
}

void setPositionAll(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, double IDs[18], double Des_Pos[20]){
    for (int i=0; i<18; i++){
    setPosition(portHandler,packetHandler,IDs[i], Des_Pos[i]);
    }
    delay(Des_Pos[18]*1000);
}

void getPositionAll(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, double IDs[18], double qPresent[18]){
    uint16_t pos;
    for (int i=0; i<18; i++){
    pos = getPosition(portHandler,packetHandler, IDs[i]);
    /*
    Serial.print("[ID:");
    Serial.print(IDs[i]);
    Serial.print("] Present Position: ");*/
    Serial.print(pos);
    Serial.print(" ");
    qPresent[i] = pos;
    }
  Serial.println("");
}

void setPositionSync(dynamixel::GroupSyncWrite groupSyncWritePos,dynamixel::PacketHandler *packetHandler, double IDs[18], double Des_Pos[20]){
  bool dxl_addparam_result = false;   
  uint8_t param_goal_position[2];
  
  for (int i=0; i<18; i++){
    param_goal_position[0] = DXL_LOBYTE(Des_Pos[i]);
    param_goal_position[1] = DXL_HIBYTE(Des_Pos[i]);
    dxl_addparam_result = groupSyncWritePos.addParam(IDs[i], param_goal_position);

      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:");
        Serial.print(IDs[i]); 
        Serial.println("groupSyncWrite addparam failed");
        groupSyncWritePos.clearParam();
      }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWritePos.txPacket();
    if (dxl_comm_result != COMM_SUCCESS){Serial.print(packetHandler->getTxRxResult(dxl_comm_result));}
    // Clear syncwrite param
    groupSyncWritePos.clearParam();
  delay(Des_Pos[18]*1000);
}

void setVelocitySync(dynamixel::GroupSyncWrite groupSyncWriteVel, dynamixel::PacketHandler *packetHandler,double IDs[18], double Des_Vel[18]){
  bool dxl_addparam_result = false;   
  uint8_t param_goal_velocity[2];
  
  for (int i=0; i<18; i++){
    param_goal_velocity[0] = DXL_LOBYTE(Des_Vel[i]);
    param_goal_velocity[1] = DXL_HIBYTE(Des_Vel[i]);
    dxl_addparam_result = groupSyncWriteVel.addParam(IDs[i], param_goal_velocity);

      if (dxl_addparam_result != true)
      {
        Serial.print("[ID:");
        Serial.print(IDs[i]); 
        Serial.println("] groupSyncWrite addparam failed");
        groupSyncWriteVel.clearParam();
      }
    }
    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteVel.txPacket();
    if (dxl_comm_result != COMM_SUCCESS){Serial.print(packetHandler->getTxRxResult(dxl_comm_result));}
    // Clear syncwrite param
    groupSyncWriteVel.clearParam();
}

void getPositionBulk(dynamixel::GroupBulkRead groupBulkRead, dynamixel::PacketHandler *packetHandler, double IDs[18]){
  dxl_comm_result = groupBulkRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    Serial.println(packetHandler->getTxRxResult(dxl_comm_result));
  }

   for (int i=0; i<18; i++){
    int dxl_present_position = groupBulkRead.getData(IDs[i], 36, LEN_MX_PRESENT_POSITION);
    Serial.print("[ID:");
    Serial.print(IDs[i]); 
    Serial.print("] Present Position: ");
    Serial.println(dxl_present_position); 
   } 
}

#endif
