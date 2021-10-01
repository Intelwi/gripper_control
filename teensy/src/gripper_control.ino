#include <Dynamixel2Arduino.h>
#include <Arduino_JSON.h>


#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_IDL = 1;
const uint8_t DXL_IDR = 2;
const float DXL_PROTOCOL_VERSION = 1.0;

/*Dynamixel Library*/
int k=0;
int grip_open[2]={450,574};
int grip_close[2]={512,512};
int comodin0[2]={522,502};
int comodin1[2]={532,492};
int comodin2[2]={542,482};
int comodin3[2]={552,472};
int comodin4[2]={562,462};
int comodin5[2]={572,452};
char rxChar;



unsigned long ax_bps;

// Set Port baudrate to 9600bps. This has to match with DYNAMIXEL baudrate.
int serial_bps = 9600;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

/*Continous sensing*/
int actual_value[2]={512,512};

int val =0;

bool set_up_dynamixel()
{
  bool result = false;
  
  ax_bps = dxl.getPortBaud();

  // Set Port baudrate. Should be 57600.
  dxl.begin(ax_bps);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Get DYNAMIXEL information
  result = dxl.ping(DXL_IDL);

  if (result)
  {
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_IDL);
    dxl.setOperatingMode(DXL_IDL, OP_POSITION);
    dxl.torqueOn(DXL_IDL);

    DEBUG_SERIAL.print("\n Connected to the motors! \n");
      
    DEBUG_SERIAL.print("\n Detected dynamixel baudrate:\n");
    DEBUG_SERIAL.print(ax_bps);

    return result;
  }

  DEBUG_SERIAL.print("\n Can not connect to the motors! \n");
  return result;
}
  
void gripper_action(int desired_action[])
{
  for(k=1;k<=2;k++)
  {
    dxl.setGoalPosition(k, desired_action[k-1]);
  }

  k=0;
  actual_value[0]=desired_action[0];
  actual_value[1]=desired_action[1];
}

void setup() {
  DEBUG_SERIAL.begin(serial_bps);
  bool result = false;

  while (!result)
  {
    result = set_up_dynamixel();
    delay(500);
  }
}

void loop() {
  //  DEBUG_SERIAL.println("I'm in loop");
  if(DEBUG_SERIAL.available() > 0)
  {
    rxChar = DEBUG_SERIAL.read();
    if(rxChar == '1'){
      digitalWrite(13, HIGH); 
      gripper_action(grip_close);
      }
    else if(rxChar == '2') gripper_action(grip_open);
    else if(rxChar == '3') gripper_action(comodin0);
    else if(rxChar == '4') gripper_action(comodin1);
    else if(rxChar == '5') gripper_action(comodin2);
    else if(rxChar == '6') gripper_action(comodin3);
    else if(rxChar == '7') gripper_action(comodin4);
//  else if(rxChar == '8') gripper_action_threshold();
//  else if(rxChar == '9') activate_slipage_detector();
  }
  //gripper_action(grip_close);
  //delay(500);
  //gripper_action(grip_open);
  delay(500);
}
