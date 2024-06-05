#ifndef DXL_MX
#define DXL_MX

/*  DYNAMIXEL MOTOS MX SERIES
 *   
 *  Available functions:
 * 
 *  void startCom().....................Initializes communication...................................................OK
 *  void pingMotor()....................Checks communications.......................................................OK   
 *  void stopcom()......................Ends communication..........................................................OK
 *  void setLED().......................Controls LED's state........................................................OK
 *
 *  void set/getTorque()................Set and Get torque value....................................................OK
 *  void set/getVelocity()..............Set and Get speed value.....................................................OK
 *  void set/getPosition()........ .....Set and Get position value..................................................OK
 *  
 *  Motor model: https://emanual.robotis.com/docs/en/dxl/mx/mx-106/
 *  Motor model: https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
 */

#include <DynamixelSDK.h>

#define DEVICENAME              "OpenCR_DXL_Port"  // Nombre simbólico, se debe checar el puerto.
#define PROTOCOL_VERSION        1.0  // Protocolo 1.
#define BAUDRATE                2000000         // Que baudrate usamos.


int dxl_comm_result = COMM_TX_FAIL;

// startCom OK SERIE MX
bool startCom(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int baudrate) {

  if (portHandler->openPort()) {
    Serial.print("Succeeded to open the port!\n");
  } else {
    Serial.print("Failed to open the port!\n");
    return false;
  }

  if (portHandler->setBaudRate(baudrate)) {
    Serial.print("Succeeded to change the baudrate!\n");
  } else {
    Serial.print("Failed to change the baudrate!\n");
    return false;
  }
  return true;
}

// stopCom OK SERIE MX
void stopCom(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  Serial.print("Stopping comms...");
  portHandler->closePort();
  Serial.println("Done");
}

// pingMotor OK SERIE MX
void pingMotor(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID) {
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result

  uint8_t dxl_error = 0;          // Dynamixel error
  uint16_t dxl_model_number = 0;  // Dynamixel model number

  dxl_comm_result = packetHandler->ping(portHandler, DXL_ID, &dxl_model_number, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] ping Failed ");
  } else if (dxl_error != 0) {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] ping Failed ");
  } else {
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.print("] ping Succeeded. Dynamixel model number : ");
    Serial.println(dxl_model_number);
  }
}

// setLED OK SERIE MX
void setLED(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID, uint8_t Led_State) {

  uint8_t dxl_error = 0;  // Dynamixel error
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, 25, Led_State, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    Serial.println("LED OK");
  }
}

//setTorque OK SERIE MX
void setTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID, uint8_t Torque_State) {

  uint8_t dxl_error = 0;  // Dynamixel error
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, 24, Torque_State, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
   //Serial.println("TORQUE:");
    if (Torque_State == 0) {
      //Serial.println("DISABLED -> 0");
    } else if (Torque_State == 1) {
      //Serial.println("ENABLED -> 1");
    }
  }
}

//getTorque OK SERIE MX
uint8_t getTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID) {

  uint8_t dxl_error = 0;  // Dynamixel error
  uint8_t data;
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, 24, &data, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    if (data == 1) {
      Serial.println("CAUTION: TORQUE ENABLED -> 1");
    } else if (data == 0) {
      Serial.println("TORQUE DISABLED -> 0");
    }
  }
  return data;
}

//setVelocity OK SERIE MX
void setVelocity(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID, uint16_t Wheel_State) {
  uint8_t dxl_error = 0;                                                                              // Dynamixel error
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, 32, Wheel_State, &dxl_error);  
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    Serial.println("Velocity OK");
  }
}

// getVelocity OK SERIE MX
uint16_t getVelocity(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID) {
  int dxl_comm_result = COMM_TX_FAIL;  // Resultado de la comunicación
  uint8_t dxl_error = 0;               // Error de Dynamixel
  uint16_t dxl_present_velocity = 0;   // Velocidad actual del Dynamixel

  // Leer la velocidad actual del Dynamixel
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, 32, &dxl_present_velocity, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Read Velocity Failed ");
  } else if (dxl_error != 0) {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Error Reading Velocity ");
  } else {
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.print("] Current Velocity: ");
    Serial.println(dxl_present_velocity);
    return dxl_present_velocity;
  }
}

//setPosition OK SERIE MX
void setPosition(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID, uint16_t Des_Pos) {
  uint8_t dxl_error = 0;  // Dynamixel error
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, 30, Des_Pos, &dxl_error); //Tipo de dato incorrecto
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    Serial.println("setPosition OK");
  }
}

//getPosition OK SERIE MX
uint16_t getPosition(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint16_t data_r_p;
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, 30, &data_r_p, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    //Serial.println("getPosition OK");
    return data_r_p;
  }
}

// getModel OK SERIE MX
void getModel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t DXL_ID) {

  uint16_t Model_Number = 0;
  uint8_t dxl_error = 0;  // Dynamixel error
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, 0, &Model_Number, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Comm Failed ");
  } else if (dxl_error != 0) {
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    Serial.print("[ID:");
    Serial.print(DXL_ID);
    Serial.println("] Packet Failed ");
  } else {
    Serial.print("Model Number: ");
    Serial.println(Model_Number);
  }
}

#endif
