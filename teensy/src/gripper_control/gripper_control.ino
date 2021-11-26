#include <Dynamixel2Arduino.h>
#include <Arduino_JSON.h>


#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_IDL = 1;
const uint8_t DXL_IDR = 2;
const float DXL_PROTOCOL_VERSION = 1.0;

const byte numChars = 2000;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

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

JSONVar GLOBAL_MESSAGE; 

unsigned long ax_bps;

// Set Port baudrate to 9600bps. This has to match with DYNAMIXEL baudrate.
int serial_bps = 9600;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

/*Continous sensing*/
int actual_value[2]={512,512};

int val =0;

//########################################################

JSONVar get_status_message(){
  /*
   Form of the status message:
  [{
     "id":0,
     "current":0,
     "torque":true,
     "velocity":0,
     "position":0,
     "baudrate":57600,
     "pwm":0
    },
    {
      "id":1,
      "current":0,
      "torque":true,
      "velocity":0,
      "position":0,
      "baudrate":57600,
      "pwm":0
     }
   ]
   */
   
  JSONVar status_message;
  JSONVar dynamixel1;
  JSONVar dynamixel2;

  dynamixel1["id"] = 0;
  dynamixel1["current"] = 0.0;
  dynamixel1["torque"] = true;
  dynamixel1["velocity"] = 0.0;
  dynamixel1["position"] = 0.0;
  dynamixel1["pwm"] = 0.0;
  dynamixel1["baudrate"] = ax_bps;
  status_message[0] = dynamixel1;

  dynamixel2["id"] = 1;
  dynamixel2["current"] = 0.0;
  dynamixel2["torque"] = true;
  dynamixel2["velocity"] = 0.0;
  dynamixel2["position"] = 0.0;
  dynamixel2["pwm"] = 0.0;
  dynamixel2["baudrate"] = ax_bps;
  status_message[1] = dynamixel2;

  return status_message;
}

bool set_setting(JSONVar gripper_command){
  /*
   Form of the expected command message:
  [{
     "id":0,
     "current":0,
     "torque":true,
     "velocity":0,
     "position":0,
     "pwm":0
    },
    {
      "id":1,
      "current":0,
      "torque":true,
      "velocity":0,
      "position":0,
      "pwm":0
     }
   ]
   */
  int command[2]={gripper_command[0]["position"], gripper_command[1]["position"]};//{512, 512};//{gripper_command[0]["position"], gripper_command[1]["position"]};

  gripper_action(command);
  //DEBUG_SERIAL.print(command[0], command[1]);
  
  return true;
  }

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

// https://forum.arduino.cc/t/serial-input-basics-updated/382007
int recvWithEndMarker() {
    static byte ndx = 0;
    for(int i=0; i<numChars; i++)
    {
      receivedChars[i] = ' ';
    }
    
    char end_char;
    char endMarker = ']';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        receivedChars[ndx] = rc;
        //DEBUG_SERIAL.print(rc);
        //DEBUG_SERIAL.print(receivedChars[ndx]);
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
        end_char = rc;

        if (rc == endMarker) {
          newData = true;
          //DEBUG_SERIAL.print(receivedChars);
          return end_char;
        }
    }
}

//########################################################

void setup() {
  DEBUG_SERIAL.setTimeout(5000);
  DEBUG_SERIAL.begin(serial_bps);
  bool result = false;

  while (!result)
  {
    result = set_up_dynamixel();
    delay(500);
  }
}

void loop() {
  if(DEBUG_SERIAL.available() > 0)
  {
    char end_char = recvWithEndMarker();

    if (newData == true) {
      newData = false;
    
      String rx = receivedChars;
      Serial.print(rx);
  
      JSONVar GLOBAL_MESSAGE = JSON.parse(rx);
       
      if (JSON.typeof(GLOBAL_MESSAGE) == "undefined") {
        Serial.println("Parsing input failed!");
      }
      else{
        Serial.print(JSON.stringify(GLOBAL_MESSAGE));
      }
      set_setting(GLOBAL_MESSAGE);
    }
  }
  
  delay(500);

  JSONVar status_message = get_status_message();
  
  String status_message_str = JSON.stringify(status_message);

  //DEBUG_SERIAL.print(status_message_str);
  //DEBUG_SERIAL.print("\n");
  
  delay(500);
}
