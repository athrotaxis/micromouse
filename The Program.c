#include <MPU6050.h>
#include <iostream>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include <math.h>

//PIN DEFINITIONS
//Sensors
#define XSHUT_FRONT 14
#define XSHUT_LEFT 4
#define XSHUT_RIGHT 15

#define FRONT_ADDRESS 0x29
#define LEFT_ADDRESS 0x30
#define RIGHT_ADDRESS 0x31

//MOTOR PINS
#define LEFT_MOTOR_CTRL 16
#define LEFT_MOTOR_PWM 12
#define RIGHT_MOTOR_CTRL 13
#define RIGHT_MOTOR_PWM 26

//ENCODER PINS
#define LEFT_ENCODER_PIN 35
#define RIGHT_ENCODER_PIN 34

#define BUTTON_PIN 27

//DISTANCE RELATED TO WHEEL ENCODERS
#define PULSES_PER_REVOLUTION 12

//TIME THRESHOLD BEFORE WHEEL IS CONSIDERED STOPPED
#define STOPPED_THRESHOLD 100000

//DEFINING ACCEPTABLE ERRORS
#define ACCEPTABLE_HEADING_ERROR 1 //mm

//Defining some things for l8er
#define base_speed 100
#define CELL_LENGTH 18
#define mm_per_tick (32 * PI) / 12 //mm

//Defining distances so that the mouse knows if the wall it is detecting is the wall associated with the cell its in or not
#define FRONT_WALL_THRESHOLD 5
#define SIDE_WALL_THRESHOLD 5
#define MOMENT_OF_TRUTH_COOLDOWN 1000 //in milliseconds
#define THRESHOLD_CELLCENTRE_X 0.1
#define THRESHOLD_CELLCENTRE_Y 0.1
//#define WALL_DISTANCE 2.45 i think this is wrong leave for now

#define TARGET_SPEED 100 //placeholder
#define TARGET_ANGULAR_SPEED 50 //placeholder

MPU6050 gyro;
SFEVL53L1X sensorFront, sensorLeft, sensorRight;

//PID control stuff, defining the type
struct PIDStuff {
  float Kp, Ki, Kd;
  float integral = 0;
  float last_error = 0;
};

PIDStuff turn_pid = {1, 1, 1};
PIDStuff forward_pid = {1, 1, 1}; 


//STATES & SUCH
enum State {
  StateFinished,
  StateForward,
  StateTurnLeft,
  StateTurnRight,
  StateMOT,
  StateUTurn
};

State current_state = StateForward;

enum Event{
  EventBegin,
  EventFinish,
  EventMomentOfTruth,
  EventDoneTurning,
  EventNone
};

enum Direction {
  FRONT,
  LEFT, 
  RIGHT
};

//Defining variables

unsigned long last_gyro_time = 0; // used for tracking the previous time in terms of the arduino clock that the gyro last measured at
float gyro_z_offset = 0; //rotational velocity gyro reads while stationary
float target_heading = 0; //the heading the micromouse is supposed to travel at

float current_yaw_rate = 0.0;
float current_heading = 0.0;


//----------------------logic stuff----------------------------------------------
#define MAZE_DIMENSION 8

uint8_t weight [MAZE_DIMENSION][MAZE_DIMENSION]; //holds the weight of each cell

//printing (the mms program is lowkey basic and just reads what is printed to stderr to decide what to do)
void log2(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

//data structure containing the possible positions of walls
typedef struct walls { 
    bool north;
    bool east;
    bool south;
    bool west;
} walls;

//Create a 2D array where each cell contains whether there is a wall to the noth, east, south and west of that cell
walls wall_location[MAZE_DIMENSION][MAZE_DIMENSION];

//Initialise the mouse's current direction & position
typedef enum directions {NORTH, EAST, SOUTH, WEST} directions;
enum directions mouse_direction = NORTH;


float mouse_x = 168/2;
float mouse_y = 166/2;

//store cell locations (x, y coords) to iterate thru
typedef struct {
    int x, y;
} Cell;

uint8_t center_low = (MAZE_DIMENSION/2)-1;
uint8_t center_high = MAZE_DIMENSION/2;

bool visited[MAZE_DIMENSION][MAZE_DIMENSION];

bool wall_in_direction(walls w, int d) {
    switch(d) {
        case 0: return w.north;
        case 1: return w.east;
        case 2: return w.south;
        case 3: return w.west;
    }
    return false;
}

//Make queue (a data type required for BFS)
#define QUEUE_SIZE 256
Cell queue[256];

//queues are a data type in other languages (eg python which this program is based on).
//to use in C have to have function to enqueue (add task to the end of the queue) and dequeue (remove first task from queue)
void enqueue(Cell c, int* tail) 
{ 
    queue[(*tail)++ % QUEUE_SIZE] = c; 
}
Cell dequeue(int* head)       
{ 
    return queue[(*head)++ % QUEUE_SIZE]; 
}



//Determing the reading whilst stationary so that the gyro may be more accurate

void calibrate_gyro() {
  long sum = 0;
  for (int i = 0; i < 500; i++){
    int z_calibrate = gyro.getRotationZ();
    sum += z_calibrate;
    delay(2); //this delay along with the for loop means that it takes 500 measurments of angular velocity across the span of 1 second, can be altered to increase or decrease accuracy as needed
  }
  gyro_z_offset = (float)sum /500; //takes the average of those 500 readings to determine the defualt angular velocity reading
}

//Reading gyro (anticlockwise is positive)
#define Kg 1
void update_gyro() {
  unsigned long now = micros();
  float dt = (now - last_gyro_time) / 1e6f;
  last_gyro_time = now;

  float yaw = ((float)gyro.getRotationZ() - gyro_z_offset) / 131.0f;  // deg/s
  if (fabsf(yaw) < 0.5f) yaw = 0;

  current_yaw_rate = yaw;
  current_heading += yaw * dt;
}

//WHEEL ENCODERS
//Declaring variables
volatile int left_pulses = 0;
volatile unsigned long last_left_pulse_time = 0;
volatile unsigned long left_pulse_time = 0;
volatile int right_pulses = 0;
volatile unsigned long last_right_pulse_time = 0;
volatile unsigned long right_pulse_time = 0;

//Interrupt functions
//might be too much lines in this
void left_encoder_addition() {
  unsigned long present = micros();
  left_pulse_time = present - last_left_pulse_time;
  last_left_pulse_time = present;
  left_pulses++;
}

void right_encoder_addition() {
  unsigned long present = micros();
  right_pulse_time = present - last_right_pulse_time;
  last_right_pulse_time = present;
  right_pulses++;
}

float left_speed = 0.0f;
float right_speed = 0.0f;

float wheel_speed(volatile unsigned long &pulse_time, volatile unsigned long &last_pulse_time) {
  noInterrupts();
  unsigned long interval_snapshot = pulse_time;
  unsigned long last_time_snapshot = last_pulse_time;
  interrupts();

  unsigned long time_since_last_pulse = micros() - last_time_snapshot;

  if (time_since_last_pulse > STOPPED_THRESHOLD) {
    return 0.0f; // no pulse in a while — actually stopped
  }
  if (interval_snapshot == 0) {
    return NAN;
  }
  return mm_per_tick / (interval_snapshot / 1e6f);
}

void update_speeds() {
  float left_calc  = wheel_speed(left_pulse_time,  last_left_pulse_time);
  float right_calc = wheel_speed(right_pulse_time, last_right_pulse_time);

  if (!isnan(left_calc))  left_speed  = left_calc;
  if (!isnan(right_calc)) right_speed = right_calc;
}

float forward_pwm_calc(float dt) {
  update_speeds();                                
  float avg_speed = (left_speed + right_speed) / 2.0f;
  if (current_state == StateTurnLeft || current_state == StateTurnRight) {
    return update_PID(forward_pid, 0, avg_speed, dt);
  }
  return update_PID(forward_pid, TARGET_SPEED, avg_speed, dt);
}

void reset_pid(PIDStuff &pid) {
  pid.integral = 0;
  pid.last_error = 0;
}

float update_PID(PIDStuff &pid, float target, float actual, float dt) {
  if (dt <= 0.0001f) return 0; // prevents derivative term spiking

  float error = target - actual;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -100.0f, 100.0f);  // requires tuning

  float derivative = (error - pid.last_error) / dt;
  pid.last_error = error;

  return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

#define Kwall 1
#define WALL_DISTANCE 5.95 //cm
//uses sensor readings to feed into PD controller
float sense_lr (){
  sensorLeft.startRanging();
  sensorRight.startRanging();

  int l_dist = sensorLeft.getDistance();
  int r_dist = sensorRight.getDistance();

  int dist_wall = r_dist - l_dist;
  if (abs(dist_wall) > WALL_DISTANCE){
    int cells_off = round((dist_wall - WALL_DISTANCE)/CELL_LENGTH);  
    if (dist_wall < 0){
      dist_wall += cells_off * CELL_LENGTH;
    }
    else {
      dist_wall -= cells_off * CELL_LENGTH;
    }
  }
  return Kwall * dist_wall;
}

float turn_pwm (State state, float dt){
  if(state == StateTurnLeft){
    return update_PID(turn_pid, -TARGET_ANGULAR_SPEED, current_yaw_rate, dt);
  }
  else if(state == StateTurnRight || state == StateUTurn){
    return update_PID(turn_pid, TARGET_ANGULAR_SPEED, current_yaw_rate, dt);
  }
  else{
  return update_PID(turn_pid, sense_lr(), current_yaw_rate, dt); //maybe look at seperating all the different time_elapsed's later 
  }
}

void motors(int ctrl_pin, int pwm_pin, int pwm){
  if (pwm >= 0){
    digitalWrite(ctrl_pin, HIGH);
  }
  else{
    digitalWrite(ctrl_pin, LOW);
  }
  analogWrite(pwm_pin, constrain(abs(pwm), 0, 255));
}

void new_pwm() {
  float dt = time_passed();       
  float turn_correction = turn_pwm(current_state, dt);
  float forward_pwm = forward_pwm_calc(dt);

  int left_pwm  = base_speed + turn_correction + forward_pwm;
  int right_pwm = base_speed - turn_correction + forward_pwm;

  motors(LEFT_MOTOR_CTRL, LEFT_MOTOR_PWM, left_pwm);
  motors(RIGHT_MOTOR_CTRL, RIGHT_MOTOR_PWM, right_pwm);
}

//function used for determing the time between speed adjustments
unsigned long last_time = 0;
float time_passed() {
  unsigned long now = micros();
  float time_its_been = (now - last_time) / 1e6f;
  last_time = now;
  return time_its_been;
}
//function for driving forward

float forward_distance_travelled = 0.0f; //variable for keeping track of distance travelled

float read_right_sensor(){
  while(!sensorRight.checkForDataReady()) delay(1);
  int distance_right = sensorRight.getDistance();
  sensorRight.clearInterrupt();
  sensorRight.stopRanging();
  return (1/1.55)*(distance_right+52.143);
}

float read_left_sensor(){
  while(!sensorLeft.checkForDataReady()) delay(1);
  int distance_left = sensorLeft.getDistance();
  sensorLeft.clearInterrupt();
  sensorLeft.stopRanging();
  return (1/1.55)*(distance_left+52.143);
}

float read_front_sensor(){
  while(!sensorFront.checkForDataReady()) delay(1);
  int distance_front = sensorFront.getDistance();
  sensorFront.clearInterrupt();
  sensorFront.stopRanging();
  return (1/1.55)*(distance_front+52.143);
}

bool isWall(Direction direction) {
  sensorFront.startRanging();
  sensorLeft.startRanging();
  sensorRight.startRanging();

  if (direction == FRONT) {
    if (read_front_sensor() < FRONT_WALL_THRESHOLD) {
      return true;
    }
  }
  if (direction == LEFT) {
    if (read_left_sensor() < SIDE_WALL_THRESHOLD) {
      return true;
    }
  }
  if (direction == RIGHT) {
    if (read_right_sensor() < SIDE_WALL_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void update_walls(){
  int x = (int)(lroundf(mouse_x));
  int y = (int)(lroundf(mouse_y));
  if (isWall(FRONT)){
    if (mouse_direction == NORTH) {
            wall_location[x][y].north = true;
            //API_setWall(x, y, 'n');
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].east = true;
            //API_setWall(x, y, 'e');
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true;  }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            //API_setWall(x, y, 's');
            if (y > 0) { wall_location[x][y-1].north = true;  }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            //API_setWall(x, y, 'w');
            if (x > 0) { wall_location[x-1][y].east = true;  }
        }
  }
  if (isWall(LEFT)){
    if (mouse_direction == NORTH) {
            wall_location[x][y].west = true;
            if (x > 0) { wall_location[x-1][y].east = true; }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].north = true;
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].east = true;
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true;  }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].south = true;
            if (y > 0) { wall_location[x][y-1].north = true;  }
        }
  }
  if(isWall(RIGHT)){
     if (mouse_direction == NORTH) {
            wall_location[x][y].east = true;
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true;  }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].south = true;
            if (y > 0) { wall_location[x][y-1].north = true;  }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].west = true;
            if (x > 0) { wall_location[x-1][y].east = true; }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].north = true;
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; }
        }
  }
}

//Function that holds the logic for turning and heading reassignment
void flood_fill(){
  // Reset all weights to 255 before recalculating (its not set to 0 as thats what the center squares are)
    for (int i = 0; i < MAZE_DIMENSION; i++) {
        for (int j = 0; j < MAZE_DIMENSION; j++) {
            weight[i][j] = 255;
        }
    }

    int head = 0, tail = 0; //lowkey forgot but at one point in my life i understood this

    for (uint8_t i = 7; i < 9; i++) { //set center goals to have weight of 0
        for (uint8_t j = 7; j < 9; j++) {
            weight[i][j] = 0;
            queue[tail++] = (Cell){i, j};
        }
    }
    for (uint8_t i = center_low; i <= center_high; i++) {
      for (uint8_t j = center_low; j <= center_high; j++) {
        weight[i][j] = 0;
        enqueue((Cell){i, j}, &tail);
      }
    }
    
    
    //4 possible directions of movement of mouse which are used to change the x and y coordinate of the locaiton being looked at
    int dx[] = {0, 1, 0, -1}; 
    int dy[] = {1, 0, -1, 0};

    /*
    BFS algorithm
    From a Python program by @alfredjoejr on Github: https://github.com/alfredjoejr/mms-python/blob/main/Main.py
    Translated into C using Claude
    */
    while (head != tail) {
        Cell c = dequeue(&head);
        uint8_t current_weight = weight[c.x][c.y];

        for (int d = 0; d < 4; d++) {
            int nx = c.x + dx[d];
            int ny = c.y + dy[d];
            if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
                if (!wall_in_direction(wall_location[c.x][c.y], d)) {
                    if (current_weight < 255 && weight[nx][ny] > current_weight + 1) {
                        weight[nx][ny] = current_weight + 1;
                        enqueue((Cell){nx, ny}, &tail);
                    }  
                }
            }
        }
    }
} 

void stop(){
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

long last_left_pulses = 0;
long last_right_pulses = 0;

//function for dead reckoning
void update_position(){
  noInterrupts();
  int left_snapshot = left_pulses;
  int right_snapshot = right_pulses;
  interrupts();

  int left_delta  = left_snapshot  - last_left_pulses;
  int right_delta = right_snapshot - last_right_pulses;

  last_left_pulses  = left_snapshot;
  last_right_pulses = right_snapshot;

  float pulses = (left_delta + right_delta) / 2;
  float distance_travelled = (pulses * mm_per_tick) / 160;
  dead_reckoning(current_state, distance_travelled);
}

void dead_reckoning(State state, float distance){
  if (state == StateUTurn || state == StateTurnLeft || state == StateTurnRight){
    return;
  }
  else if (mouse_direction == NORTH){
    mouse_y += distance;
  }
  else if (mouse_direction == EAST){
    mouse_x += distance;
  }
  else if (mouse_direction == SOUTH){
    mouse_y -= distance;
  }
  else if (mouse_direction == WEST){
    mouse_x -= distance;
  }
}

void drive(){
  //probs need some stuff to do with the dead reckoning here at some point
  update_position();
  new_pwm();
}

//used to figure out whether ts turn is done
bool turn_done() {
  float heading_error = fabs(target_heading - current_heading);
  return heading_error < ACCEPTABLE_HEADING_ERROR; 
}

unsigned long momentOfTruthAt = 0;
Event update_event(){
  int cx = (int)lroundf(mouse_x);
  int cy = (int)lroundf(mouse_y);
  if ((cx == center_low || cx == center_high) &&
      (cy == center_low || cy == center_high)) {
    return EventFinish;
  }
  
  //center
  } if ((mouse_x-(float)((int)(mouse_x)))<THRESHOLD_CELLCENTRE_X) {
    if ((mouse_y-(float)((int)(mouse_y)))<THRESHOLD_CELLCENTRE_Y) {
      if(millis() - momentOfTruthAt > MOMENT_OF_TRUTH_COOLDOWN) { //we didnt already just have a moment of truth 
        // Set a flag to indicate we've just had a moment of truth
        momentOfTruthAt = millis();
        return EventMomentOfTruth;
      }
    }
  }
  if ((current_state == StateTurnLeft || current_state == StateTurnRight || current_state == StateUTurn) && (turn_done())) {
      current_heading = roundf(current_heading / 90.0f) * 90.0f; // returns the heading value to a 90 degree multiple, the bandaid method fr
      return EventDoneTurning;
    }
  return EventNone;
}

State find_best_step()
{
    int min_val = 255;
    int best_dir = -1; //forgot

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    // Check all 4 neighbours to see which is a) not blocked by a wall and b) has the lowest weight
    for (int d_check = 0; d_check < 4; d_check++) {
        int nx = (int)(lroundf(mouse_x)) + dx[d_check];
        int ny = (int)(lroundf(mouse_y)) + dy[d_check];
        if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
            // If path is clear in memory
            if (!wall_in_direction(wall_location[(int)(lroundf(mouse_x))][(int)(lroundf(mouse_y))], d_check)) {
                if (weight[nx][ny] < min_val || (weight[nx][ny] == min_val && d_check == mouse_direction)) { // preference going straight as its faster than turning
                    min_val = weight[nx][ny];
                    best_dir = d_check;
                }
            }
        }
    }

    //prolly for the real world would change API_turnRight() to have this function (move_best_step) return which way it needs to turn
    if (best_dir != -1) {
        // Turn to face best direction
        if (best_dir == mouse_direction) {
            // already facing right way
        } else if (best_dir == (mouse_direction + 1) % 4) {
            mouse_direction = (mouse_direction + 1) % 4;
            return StateTurnRight; 
        } else if (best_dir == (mouse_direction + 3) % 4) {
            mouse_direction = (mouse_direction + 3) % 4;
            return StateTurnLeft;
        } else {
            mouse_direction = (mouse_direction + 2) % 4;
            return StateUTurn;
        }

        //change position after moving forwards 
        // if (mouse_direction == NORTH) mouse_y++;
        // if (mouse_direction == EAST)  mouse_x++;
        // if (mouse_direction == SOUTH) mouse_y--;
        // if (mouse_direction == WEST)  mouse_x--;

        //and then move this line to another function, so after having called this function and then making mouse turn appropriately it will go forwards
        return StateForward;
    }
  return current_state;
}

State transition(State state, Event event){
  switch(state){
    case StateForward:
      if(event == EventFinish) return StateFinished;
      if(event == EventMomentOfTruth) return StateMOT;
      drive();
      break;
    case StateFinished:
      stop();
      break;
    case StateMOT:{
      update_walls();
      flood_fill();
      if(event == EventFinish) return StateFinished;
      State next_step = find_best_step();
      if(next_step == StateTurnLeft){
        target_heading = target_heading + 90;
      }
      if(next_step == StateTurnRight){
        target_heading = target_heading - 90;
      }
      if(next_step == StateUTurn){
        target_heading = target_heading + 180;
      }
      return next_step;
    }   
      break;
    case StateTurnLeft:
      if(event == EventFinish) return StateFinished;
      if(event == EventDoneTurning) return StateForward;
      drive();
      break;
    case StateTurnRight:
      if(event == EventFinish) return StateFinished;
      if(event == EventDoneTurning) return StateForward;
      drive();
      break;
    case StateUTurn:
      if(event == EventFinish) return StateFinished;
      if(event == EventDoneTurning) return StateForward;
      drive();
      break;
  }
  return state;
}

void wait_for_button() {
  Serial.println("Press the button to start");
  while (digitalRead(BUTTON_PIN) == LOW)  delay(10);  // if it's already held, wait for release
  while (digitalRead(BUTTON_PIN) == HIGH) delay(10);  // wait for the press
  delay(50);                                           // debounce
  while (digitalRead(BUTTON_PIN) == LOW)  delay(10);  // wait for release
}

void setup() {

  Wire.begin();
  //IMU
  gyro.initialize();
  calibrate_gyro();

  //SETTING UP MOTORS
  pinMode(LEFT_MOTOR_CTRL, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_CTRL, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  //Setting up sensors to output and powering them down
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  //Individually powering on sensors and assigning them their appropriate address
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  sensorFront.begin();

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  sensorLeft.begin();
  sensorLeft.setI2CAddress(LEFT_ADDRESS);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  sensorRight.begin();
  sensorRight.setI2CAddress(RIGHT_ADDRESS);

  //creating the interrupts for the wheel encoder functions
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), left_encoder_addition, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), right_encoder_addition, RISING);

  log2("Running...");
    char buf[4];  //msvc quirk, not sure if relevant for arduino

  //Initialise maze arrays
  for (uint8_t i=0; i < MAZE_DIMENSION; i++) {
        for (uint8_t j=0; j < MAZE_DIMENSION; j++) {
            weight[i][j] = 255; //initialise to infinity for some reason not sure?
            wall_location[i][j].north = false;
            wall_location[i][j].east = false;
            wall_location[i][j].south = false;
            wall_location[i][j].west = false;
        }
    }
    
    //Set center positions
    weight[center_low][center_low]   = 0;
    weight[center_low][center_high]  = 0;
    weight[center_high][center_low]  = 0;
    weight[center_high][center_high] = 0;

    for (int i = 0; i < MAZE_DIMENSION; i++) {
        for (int j=0; j < MAZE_DIMENSION; j++) {
            visited[i][j] = false; //initialise array of visited cells
        }
    }

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // assumes the button connects the pin to GND when pressed

  wait_for_button();

  delay(1000);         // hands off the mouse before the gyro calibrates
  calibrate_gyro();    // move the call here from the top of setup()

  // wipe anything that accumulated while it was waiting
  noInterrupts();
  left_pulses = 0;
  right_pulses = 0;
  interrupts();
  last_left_pulses = 0;
  last_right_pulses = 0;
  current_heading = 0;
  target_heading = 0;

  last_gyro_time = micros();
  last_time = micros();
}


void loop() {
  update_gyro();

  State next = transition(current_state, update_event());
  if (next != current_state) {
    reset_pid(turn_pid);
    reset_pid(forward_pid);
  }
  current_state = next;
}

  
  

//Im tired ill do more later
//ok