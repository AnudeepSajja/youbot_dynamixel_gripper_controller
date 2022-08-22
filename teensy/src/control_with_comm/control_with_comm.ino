// State Machine implementation based on https://www.norwegiancreations.com/2017/03/state-machines-and-arduino-implementation/

#include <Dynamixel2Arduino.h>
#include <CircularBuffer.h>
#include <Arduino_JSON.h>

#define DXL_SERIAL   Serial1    
#define DEBUG_SERIAL Serial     
const uint8_t DXL_DIR_PIN = 2;
const uint8_t DXL_IDL = 1;
const uint8_t DXL_IDR = 2;
const float DXL_PROTOCOL_VERSION = 1.0;

/*Dynamixel Library*/
int k=0; 
int id; // Used to cycle through dynamixel IDs
const int GRIPPER_VELOCITY = 100;  // Velocity of the gripper fingers
                                  // 75 is too much and overcurrent occurs with the big objects
const int POSITION_TOLERANCE = 5;  // Fixed pos tolerance
const int SLIDING_WINDOW_SIZE = 20; // Fixed sliding window size


// Predefined states
int GRIP_OPEN[2]={450,574};
int GRIP_CLOSE[2]= {573, 448}; //{577,447};

// To store the baudrate for DYNAMIXEL
unsigned long ax_bps;
// Set Port baudrate to 9600bps.
const int SERIAL_BPS = 9600;

// We initialize an object with an open serial connection and the DIR_PIN
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Communication with PC variables
const int NUM_CHARS = 2000;
char receivedChars[NUM_CHARS];   // an array to store the received data
boolean newData = false;
int parse_error = 0;
int gripper_command = 0;

// State machine variables
enum States {GRIPPER_OPENING, GRIPPER_CLOSING, OBJECT_GRASPED, OBJECT_SLIPPED, GRIPPER_CLOSED, GRIPPER_OPEN};
uint8_t state = GRIPPER_OPEN;
long int curr_time = 0, last_time = millis();

// Loop variables
int iter = 0;
int ITER_LIMIT = 1000;

int global_pos[2] = {0};


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
  
  //ax_bps = dxl.getPortBaud();

  // Set Port baudrate. Should be 57600. Initializes Serial comms with DYNAMIXEL
  dxl.begin(1000000);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Checks the connection status of Left DYNAMIXEL.
  result_l = dxl.ping(DXL_IDL);
  if (result_l){
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_IDL);
    dxl.setOperatingMode(DXL_IDL, OP_POSITION);
    dxl.torqueOn(DXL_IDL);
  }

  // Checks the connection status of Right DYNAMIXEL.
  result_r = dxl.ping(DXL_IDR);
  
  if (result_r){
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_IDR);
    dxl.setOperatingMode(DXL_IDR, OP_POSITION);
    dxl.torqueOn(DXL_IDR);
  }
  
  // We return false if one of them doesn't work
  return result_l && result_r;
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

float new_abs(float number){
  /* Function calculating absolute value (standard abs clashes with the used libraries)
   * Input:   float number
   *          2 element Integer array of the desired final positions of each dynamixel
   */
  if(number<0)
  {
    number = -number;
  }
  return number;
}

void move_gripper_to_position(int desired_action[]){
  /* Function which sets the goal position
   * Input:   int[array] desired_action
   *          2 element Integer array of the desired final positions of each dynamixel
   */
  for(int id=1;id<=2;id++){
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
}

bool is_closed(int curr_pos[], int desired_pos[]) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
   int diff = curr_pos[0] - curr_pos[1];
  return (new_abs(desired_pos[0] - curr_pos[0]) < POSITION_TOLERANCE) && (new_abs(desired_pos[1] - curr_pos[1]) < POSITION_TOLERANCE) || diff>119;
}

bool is_close(int curr_pos[], int desired_pos[]) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
  return (new_abs(desired_pos[0] - curr_pos[0]) < POSITION_TOLERANCE) && (new_abs(desired_pos[1] - curr_pos[1]) < POSITION_TOLERANCE);
}

void get_pos(int *pos) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
  for(int id = 1; id <= 2; id++){
    pos[id-1] = dxl.getPresentPosition(id);
  }
    
}

void get_pos_degrees(int *pos) {
  /* Function to write the current position of the dynamixel motors
   * to a given array
   * Input:     [int*]
   *            Array pointer in which the values are stored
   */
  for(int id = 1; id <= 2; id++){
    pos[id-1] = dxl.getPresentPosition(id, UNIT_DEGREE);
  }
}

void simple_movement(int desired_action[]){
  // Set Goal positions for both the DYNAMIXELs
  for(int id=1;id<=2;id++)
  {
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
 }
  
void careful_movement(int desired_action[], int sliding_window_size = 10, int position_tolerance = 5, int minimum_movment = 2, int pos_offset = 2){
  /* Function to move the grippers to a particular position. In case the grippers are
   * not able to move further due to some reason, the grippers are stopped to prevent
   * overheating
   * Input:     int[array]  desired_action
   *            2 element Integer array of the desired positions of each dynamixel
   *            
   *            Optional/Default parameters:
   *            [int]       sliding_window_size
   *            The approach compares the first and last elements of the sliding window, so the size of the window controls
   *            the time between the comparison of states of the gripper to see if it has moved at all.
   *            [int]       position_tolerance
   *            This represents the amount of tolerance in the position, lower values make it more precise and higher values
   *            make it more lenient.
   *            [int]       minimum_movment
   *            This is the minimum difference in motion which should be observed so as to consider the gripper as moving
   *            [int]       pos_offset
   *            In order to tighten the grip, we move the dynamixels by this amount after it has encountered an obstruction
   *            This number indicates the amount by which the dynamixels move to tighten the grip
   */

  // Initialize buffers
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
      dir_motion[id-1] = (diff)/new_abs(diff);
    else
      dir_motion[id-1] = 0;
  }
  
  // Set Goal positions for both the DYNAMIXELs
  for(int id=1;id<=2;id++)
  {
    dxl.setGoalPosition(id, desired_action[id-1]);
  }
  
  // Continue monitoring till final position is reached for both the motors
  for( int iter_num=0; (new_abs(desired_action[0] - dxl.getPresentPosition(1)) > position_tolerance) || (new_abs(desired_action[1] - dxl.getPresentPosition(2)) > position_tolerance); iter_num++ ){
    // Add new observations to our buffers
    buffer1.push(dxl.getPresentPosition(1));
    buffer2.push(dxl.getPresentPosition(2));

    // Once we have sufficient number of observations, we start working with these
    if (iter_num > sliding_window_size){
      if((new_abs(buffer1.last() - buffer1[buffer1.size() - sliding_window_size]) < minimum_movment) || (new_abs(buffer2.last() - buffer2[buffer2.size() - sliding_window_size]) < minimum_movment)){
        int new_desired_action[2];
        new_desired_action[0] = dxl.getPresentPosition(1) + dir_motion[0]*(pos_offset + position_tolerance);
        new_desired_action[1] = dxl.getPresentPosition(2) + dir_motion[1]*(pos_offset + position_tolerance);
            
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
  /* Function to run the state machine for the gripper
   */
   
  // Stores the current position
  int curr_pos[2] = {0};
  int old_pos[2]  = {0};
  
  switch(state)
  {
    case GRIPPER_OPENING:
      // We make sure that the gripper is open
      // careful_movement(GRIP_OPEN, SLIDING_WINDOW_SIZE, POSITION_TOLERANCE);
      simple_movement(GRIP_OPEN);
      get_pos(curr_pos);

      if (gripper_command==1){
        state = GRIPPER_CLOSING; 
      }
      else if(is_close(curr_pos, GRIP_OPEN)){   // May be problematic for thin objects
        // Add code to communicate this info to the ROS system
        state = GRIPPER_OPEN;
      }
      break;
      
    case GRIPPER_OPEN:
      // We make sure that the gripper is open
      if (gripper_command==1){
        state = GRIPPER_CLOSING; 
      }
      break;
       
    case GRIPPER_CLOSING:
      /* Code for closing gripper comes here, it can either succeed or fail
       * Success -> OBJECT_GRASPED
       * Fail -> OBJECT_SLIPPED (or MISSED)
      */
      
      //DEBUG_SERIAL.println("Current State = GRIPPER_CLOSE");
      careful_movement(GRIP_CLOSE, SLIDING_WINDOW_SIZE, POSITION_TOLERANCE);
      get_pos(curr_pos);

      // Checks if we missed the object
      if(is_closed(curr_pos, GRIP_CLOSE)){   // May be problematic for thin objects
        // Add code to communicate this info to the ROS system
        state = GRIPPER_CLOSED;
      }
      else{ 
        state = OBJECT_GRASPED;
        //coping pos
        global_pos[0] = curr_pos[0];
        global_pos[1] = curr_pos[1]; 
      }
      break;
 
    case OBJECT_GRASPED:
      // Checks if object still in hand periodically
      // Transitions to GRIPPER_OPEN when external command is received or if we drop the object

      curr_time = millis();
      
      if(gripper_command == 0){
        //DEBUG_SERIAL.println("Command received, dropping item in 2 secs");
        state = GRIPPER_OPENING;
      }
      else if (curr_time - last_time > 1000){ 
        // Keep checking if object in hand every 1 secs
        //careful_movement(GRIP_CLOSE, SLIDING_WINDOW_SIZE, POSITION_TOLERANCE);
        get_pos(curr_pos);

        // Check we have dropped the object
        if(is_closed(curr_pos, GRIP_CLOSE)){
          state = GRIPPER_CLOSED;
        }
        else if(!is_close(curr_pos, global_pos)){   
          // May be problematic for thin objects
          state = GRIPPER_CLOSING;
        }
        
        last_time = millis();
      }
      break;
    
    case GRIPPER_CLOSED:
      if (gripper_command==0){
        state = GRIPPER_OPENING; 
      }
  }
}

// ----------------- //
// Communication functions //
// ----------------- //

bool rec_with_end_marker() {
  /* Function to receive the data from the serial
   *   https://forum.arduino.cc/t/serial-input-basics-updated/382007
   */
    byte ndx = 0;
    for(int i=0; i<NUM_CHARS; i++)
    {
      receivedChars[i] = ' ';
    }
    
    char end_char;
    char endMarker = '}';
    char rc;
    
    while (DEBUG_SERIAL.available() > 0 && newData == false) {
        rc = DEBUG_SERIAL.read();
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= NUM_CHARS) {
            ndx = NUM_CHARS - 1;
        }
        end_char = rc;

        if (rc == endMarker) {
          newData = true;
          return true;
        }
    }
    return false;
}

void get_command(){
  /* Function to receive the commands from the serial
   */
  parse_error = 0;
  
  if(DEBUG_SERIAL.available() > 0)
  {
    bool is_succeeded = rec_with_end_marker();

    if (newData == true) {
      newData = false;
    
      String rx = receivedChars;
  
      JSONVar message = JSON.parse(rx);
      
      gripper_command = message["command"];
      
      if (JSON.typeof(message) == "undefined") {
        parse_error = 1;
      }
    }
  }

}

void send_feedback()
{
  /* Function for sending the feedback to the serial
   */
  JSONVar feedback_message;

  if(state==GRIPPER_OPENING) feedback_message["state"] = "GRIPPER_OPENING";
  else if(state==GRIPPER_OPEN) feedback_message["state"] = "GRIPPER_OPEN";
  else if(state==GRIPPER_CLOSING) feedback_message["state"] = "GRIPPER_CLOSING";
  else if(state==OBJECT_GRASPED) feedback_message["state"] = "OBJECT_GRASPED";
  else if(state==OBJECT_SLIPPED) feedback_message["state"] = "OBJECT_SLIPPED";
  else if(state==GRIPPER_CLOSED) feedback_message["state"] = "GRIPPER_CLOSED";

  feedback_message["parsing_error"] = parse_error;
  feedback_message["last_command"] = gripper_command;

  int pos[2] = {0};
  get_pos_degrees(pos);
  
  feedback_message["left_gripper_pos"] = pos[0];
  feedback_message["right_gripper_pos"] = pos[1];
  feedback_message["diff_encoders"] = pos[0] - pos[1];

  String feedback_message_str = JSON.stringify(feedback_message);
  feedback_message_str = feedback_message_str + "\n";
  //strcat(feedback_message_str, "\n");

  DEBUG_SERIAL.print(feedback_message_str);
}

// -------------------------------- //
// Microcontroller setup and loop functions //
// -------------------------------- //
void setup(){
  DEBUG_SERIAL.begin(9600);
  bool result = false;

  // Initialize/Set up the microcontroller
  while (!result)
  {
    result = set_up_dynamixel();
    delay(500);
  }
  
  // Set default velocity
  set_velocity(GRIPPER_VELOCITY);

  simple_movement(GRIP_OPEN);
}
 

void loop(){
  if(iter<ITER_LIMIT) iter++;
  else iter=0;
  get_command();
  state_machine_run();
  send_feedback();
  
  delay(500);
}
