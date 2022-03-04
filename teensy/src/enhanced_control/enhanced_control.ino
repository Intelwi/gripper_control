// State Machine implementation based on https://www.norwegiancreations.com/2017/03/state-machines-and-arduino-implementation/

#include <Dynamixel2Arduino.h>
#include <CircularBuffer.h>

#define DXL_SERIAL   Serial1    
#define DEBUG_SERIAL Serial     
const uint8_t DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
const uint8_t DXL_IDL = 1;
const uint8_t DXL_IDR = 2;
const float DXL_PROTOCOL_VERSION = 1.0;

/*Dynamixel Library*/
int k=0; 
int id;         // Used to cycle through dynamixel IDs
int vel = 150;  //
int position_tolerance = 10;  // Fixed pos tolerance


// Predefined states
int grip_open[2]={450,574};
int grip_close[2]={562,462};

// To store the baudrate for DYNAMIXEL
unsigned long ax_bps;

// Set Port baudrate to 9600bps.
int serial_bps = 9600;

// We initialize an object with an open serial connection and the DIR_PIN
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

int val = 0;

enum States {GRIPPER_OPEN, GRIPPER_CLOSE, OBJECT_GRASPED, OBJECT_SLIPPED};
//enum Sensors_enum {NONE, SENSOR_RIGHT, SENSOR_LEFT, BOTH};


// State machine variables
uint8_t state = GRIPPER_OPEN;
bool picking_object = false;    //Do I have a command to pick an object
long int curr_time = 0, last_time = millis();


// --------------- //
// Setup functions //
// --------------- //
bool set_up_dynamixel()
{
  /* Function to check the port baudrates, set the baudrates for communication and
   * initialize both the dynamixels
   * 
   * Returns:   bool
   *            'True' if both the dynamixels are working
   */
  bool result_l = false;
  bool result_r = false;
  
  ax_bps = dxl.getPortBaud();

  // Set Port baudrate. Should be 57600. Initializes Serial comms with DYNAMIXEL
  dxl.begin(ax_bps);
  DEBUG_SERIAL.print(ax_bps);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Checks the connection status of Left DYNAMIXEL.
  result_l = dxl.ping(DXL_IDL);
  if (result_l){
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_IDL);
    dxl.setOperatingMode(DXL_IDL, OP_POSITION);
    dxl.torqueOn(DXL_IDL);

    DEBUG_SERIAL.print("\n Connected to the a motor! Motor ID: ");
    DEBUG_SERIAL.print(DXL_IDL);
    DEBUG_SERIAL.print("\n Detected dynamixel baudrate, using common baudrate:\n");
    DEBUG_SERIAL.print(ax_bps);
  }
  else{
    DEBUG_SERIAL.print("\n Can not connect to the motor");
    DEBUG_SERIAL.print(DXL_IDL);
    DEBUG_SERIAL.print("\n"); 
  }

  // Checks the connection status of Right DYNAMIXEL.
  result_r = dxl.ping(DXL_IDR);
  if (result_r){
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_IDR);
    dxl.setOperatingMode(DXL_IDR, OP_POSITION);
    dxl.torqueOn(DXL_IDR);

    DEBUG_SERIAL.print("\n Connected to the a motor! Motor ID: ");
    DEBUG_SERIAL.print(DXL_IDR);
    DEBUG_SERIAL.print("\n Detected dynamixel baudrate, using common baudrate:\n");
    DEBUG_SERIAL.print(ax_bps);
  }
  else{
    DEBUG_SERIAL.print("\n Can not connect to the motor");
    DEBUG_SERIAL.print(DXL_IDR);
    DEBUG_SERIAL.print("\n"); 
  }
  // DEBUG_SERIAL.print("\n Can not connect to the motors! \n");
  // We return false if either doesn't work
  return result_l & result_r;
}

void set_velocity(int velocity){
  /* Function to set the operating velocity
   * Temporarily switches to velocity mode and then reverts back to position mode
  */ 
  for(id=1; id<=2; id++){
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.setGoalVelocity(id, velocity);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
  }
}  


// ----------------- //
// General functions //
// ----------------- //

void move_gripper_to_position(int desired_action[]){
  /* Function which sets the goal position
   * Input:   int[array] desired_action
   *          2 element Integer array of the desired final positions of each dynamixel
   */
  for(int id=1;id<=2;id++){
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
}

bool is_close(int curr_pos[], int desired_pos[]) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
//  DEBUG_SERIAL.println(desired_pos[0]);
  return (abs(desired_pos[0] - curr_pos[0]) < position_tolerance) && (abs(desired_pos[1] - curr_pos[1]) < position_tolerance);
}

void get_pos(int *pos) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
  for(int id = 1; id <= 2; id++){
    pos[id-1] = dxl.getPresentPosition(id);
//    DEBUG_SERIAL.println();
  }
    
}

void display_position(){
  /* Function to print the current position of the DYNAMIXEL motors
   * Print Format - RAW - [0,1023]
   * The UNIT_DEGREE isn't working for some reason
   */
  char buf[100];
  sprintf(buf, "\nDynamixel motor 1:  Position: %f, \nDynamixel motor 2:  Position: %f", dxl.getPresentPosition(1), dxl.getPresentPosition(2)); 
  DEBUG_SERIAL.println(buf);
}

void display_all_variables(int loop_counter = 0, bool count = false){
  /* Function which displays the position, velocity, current and PWN of the two Dyamixels ID 1 & 2
   * Single line serial print functions result in faster execution times when compared to multiple print statements.
   * 
   * Input:     [int]   loop_counter
   *            Print a specific iteration number
   *            
   *            [bool]  count
   *            Enables printing a specific iteration number
   *            Set to 'fasle' by default
   */
  if(count){
    char buf[200];
    sprintf(buf, "\nDynamixel motor 1:  Position: %f, Velocity: %f, Current: %f, PWM: %f\nDynamixel motor 2:  Position: %f, Velocity: %f, Current: %f, PWM: %f", dxl.getPresentPosition(1), dxl.getPresentVelocity(1), dxl.getPresentCurrent(1), dxl.getPresentPWM(1), dxl.getPresentPosition(2), dxl.getPresentVelocity(2), dxl.getPresentCurrent(2), dxl.getPresentPWM(2)); 
    DEBUG_SERIAL.println(buf);  
  }
  else{
    char buf[200];
    sprintf(buf, "\n Read Instance: %d Dynamixel motor 1:  Position: %f, Velocity, Current: %f, PWM: %f: %f\nDynamixel motor 2:  Position: %f, Velocity: %f, Current: %f, PWM: %f", loop_counter, dxl.getPresentPosition(1), dxl.getPresentVelocity(1), dxl.getPresentCurrent(1), dxl.getPresentPWM(1), dxl.getPresentPosition(2), dxl.getPresentVelocity(2), dxl.getPresentCurrent(2), dxl.getPresentPWM(2)); 
    DEBUG_SERIAL.println(buf);  
  }  
}   

void display_position_and_velocity(int loop_counter = 0, bool count = false){
  /* Function which displays the position and velocity of the two Dyamixels ID 1 & 2
   * Single line serial print functions result in faster execution times when compared to multiple print statements.
   * 
   * Input:     [int]   loop_counter
   *            Print a specific iteration number
   *            
   *            [bool]  count
   *            Enables printing a specific iteration number
   *            Set to 'fasle' by default
   */
  if(count){
    char buf[150];
    sprintf(buf, "\nDynamixel motor 1:  Position: %f, Velocity: %f\nDynamixel motor 2:  Position: %f, Velocity: %f", dxl.getPresentPosition(1), dxl.getPresentVelocity(1), dxl.getPresentPosition(2), dxl.getPresentVelocity(2)); 
    DEBUG_SERIAL.println(buf);  
  }
  else{
    char buf[150];
    sprintf(buf, "\n Read Instance: %d Dynamixel motor 1:  Position: %f, Velocity: %f\nDynamixel motor 2:  Position: %f, Velocity: %f", loop_counter, dxl.getPresentPosition(1), dxl.getPresentVelocity(1), dxl.getPresentPosition(2), dxl.getPresentVelocity(2)); 
    DEBUG_SERIAL.println(buf);  
  }
}

void move_to_goal_and_give_feedback(int desired_action[]){
  /* Function to move to a goal position and return feedback at each step
   * Input:   int[array]  desired_action
   *          2 element Integer array of the desired final positions of each dynamixel
   */
  for(id=1;id<=2;id++)
  {
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
  DEBUG_SERIAL.println("Tightening grip ...");
  
  for( id=0; (abs(desired_action[0] - dxl.getPresentPosition(1)) > position_tolerance) && (abs(desired_action[1] - dxl.getPresentPosition(2)) > position_tolerance); id++ ){
    display_position_and_velocity();
  }
}

void simple_movement(int desired_action[], int sliding_window_size = 10, int position_tolerance = 5, int minimum_movment = 3, int pos_offset = 2){
  /* Function to move the grippers to a particular position. In case the grippers are
   * not able to move further due to some reason, the grippers are stopped to prevent
   * overheating
   * Input:     int[array]  desired_action
   *            2 element Integer array of the desired positions of each dynamixel
   *            
   *            Optional/Default parameters:
   *            [int]       sliding_window_size
   *            The approach compares the first and last elements of the sliding window, so the size controls the duration
   *            [int]       position_tolerance
   *            This represents the amount of tolerance in the position
   *            [int]       minimum_movment
   *            This is the minimum difference in motion which should be observed so as to consider the gripper as moving
   *            [int]       pos_offset
   *            In order to tighten the grip, we move the dynamixels by this amount after it has encountered an obstruction
   *            This number indicates the amount by which the dynamixels move to tighten the grip
   */

  // We initialize some hyper-parameters, these can be converted to function args
  
  CircularBuffer<int, 100> buffer1;
  CircularBuffer<int, 100> buffer2;    

  // We make note of the old positions of the dynamixels
  int old_pos[2];
  for(int id=1;id<=2;id++)
  {
    old_pos[id-1] = dxl.getPresentPosition(id);
  }

  // Find direction of motion of the dynamixels
  int dir_motion[2], diff;
  for(int id=1;id<=2;id++)
  {
    diff = desired_action[id-1]-old_pos[id-1];
    if (diff != 0)
      dir_motion[id-1] = (diff)/abs(diff);
    else
      dir_motion[id-1] = 0;
  }
  
  // Set Goal positions for both the DYNAMIXELs
  for(int id=1;id<=2;id++)
  {
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
  
  // Continue monitoring till final position is reached for both the motors
  for( int iter_num=0; (abs(desired_action[0] - dxl.getPresentPosition(1)) > position_tolerance) || (abs(desired_action[1] - dxl.getPresentPosition(2)) > position_tolerance); iter_num++ ){
    // Add new observations to our buffers
    buffer1.push(dxl.getPresentPosition(1));
    buffer2.push(dxl.getPresentPosition(2));

    // Once we have sufficient number of observations, we start working with these
    if (iter_num > sliding_window_size){
      if((abs(buffer1.last() - buffer1[buffer1.size() - sliding_window_size]) < minimum_movment) || (abs(buffer2.last() - buffer2[buffer2.size() - sliding_window_size]) < minimum_movment)){
        // Add code to add some more distance to get a good grip
        int new_desired_action[2];
        new_desired_action[0] = dxl.getPresentPosition(1) + dir_motion[0]*(pos_offset + position_tolerance);
        new_desired_action[1] = dxl.getPresentPosition(2) + dir_motion[1]*(pos_offset + position_tolerance);
        DEBUG_SERIAL.println("Large Object?? I'm stopping!!! :(");
        
        move_gripper_to_position(new_desired_action);
        break;
      }
    }
  }
}


// ------------- //
// State Machine //
// ------------- //
void state_machine_run() 
{ 
  // Stores the current position
  int curr_pos[2] = {0};
  int old_pos[2]  = {0};

  // Add code to set custom parameters or not
  // Set or unset the variables for gripper positions, velocities etc
  // if(SERIAL.available)
  
  switch(state)
  {
    case GRIPPER_OPEN:
      /* Michal's code of getting info from ROS system comes here
       * Any message shouldn't trigger this conditional, rather, 
       * only that which closes the gripper
      */
      DEBUG_SERIAL.println("Current State = GRIPPER_OPEN");
      // We make sure that the gripper is open
      simple_movement(grip_open);

      // Simulating a condition where we recieve a command from the ros computer
      if (random(0,5)==0){
        picking_object = true;
        if(picking_object){
          DEBUG_SERIAL.println("Closing the gripper");
          // Transition to state GRIPPER_CLOSE
          state = GRIPPER_CLOSE; 
        }
      }
      break;
       
    case GRIPPER_CLOSE:
      /* Code for closing gripper comes here, it can either succeed or fail
       * Success -> OBJECT_GRASPED
       * Fail -> OBJECT_SLIPPED (or MISSED)
      */
      
      DEBUG_SERIAL.println("Current State = GRIPPER_CLOSE");
      simple_movement(grip_close);
      get_pos(curr_pos);

      // Checks if we missed the object
      if(is_close(curr_pos, grip_close)){   // May be problematic for thin objects
        DEBUG_SERIAL.println("Seems like we dropped the object, opening the gripper again for retry");
        // Add code to communicate this info to the ROS system
        state = GRIPPER_OPEN;
      }
      else{ 
        DEBUG_SERIAL.println("Seems like we grasped the object");
        state = OBJECT_GRASPED;  
      }
      break;
 
    case OBJECT_GRASPED:
      // Checks if object still in hand periodically
      // Transitions to GRIPPER_OPEN when external command is received or if we drop the object
      
      DEBUG_SERIAL.println("Current State = OBJECT_GRASPED");

      int command_recieved;
      curr_time = millis();
      
      // ROS System command for opening should come here
      command_recieved = 1;//random(0,5);
      
      if(command_recieved == 0){
        DEBUG_SERIAL.println("Command received, dropping item in 2 secs");
        state = GRIPPER_OPEN;
        picking_object = false;
        delay(2000);
      }
      else if (curr_time - last_time > 1000){ 
        // Keep checking if object in hand every 1 secs
        simple_movement(grip_close);
        get_pos(curr_pos);

        // Check we have dropped the object
        if(is_close(curr_pos, grip_close)){   // May be problematic for thin objects
          DEBUG_SERIAL.println("Seems like we dropped the object, opening the gripper again for retry");
          // Add code to communicate this info to the ROS system
          state = GRIPPER_OPEN;
        }
        else{
          DEBUG_SERIAL.println("Object still in hand");
          // We stay in the same state
        }
        last_time = millis();
      }
      else{
        // Do nothing
      }
      break;
 
    case OBJECT_SLIPPED:
      // Sends an error message to ROS system and transitions to GRIPPER_OPEN
      DEBUG_SERIAL.println("Current State = OBJECT_SLIPPED");
      state = GRIPPER_OPEN;
      picking_object = false;
      break;

  }
}


// -------------------------------- //
// Arduino setup and loop functions //
// -------------------------------- //
void setup(){
  DEBUG_SERIAL.begin(serial_bps);
  bool result = false;

  // Initialize/Set up the dynamixel
  while (!result)
  {
    result = set_up_dynamixel();
    delay(500);
  }
  
  // Set default velocity
  set_velocity(45); //75 is too much and overcurrent occurs with the big objects
}
 

void loop(){
  state_machine_run();
  delay(100);
}
