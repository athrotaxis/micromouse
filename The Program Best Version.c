#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include <math.h>
#include "types.h"

/*
Best version of The Program after work during the final day before the competition.
*/

// ---------------- PINS ----------------
#define XSHUT_FRONT 14
#define XSHUT_LEFT 4
#define XSHUT_RIGHT 15

// Front must NOT stay at the default 0x29, or the next sensor clashes with it
#define FRONT_ADDRESS 0x32
#define LEFT_ADDRESS 0x30
#define RIGHT_ADDRESS 0x31

#define LEFT_MOTOR_CTRL 16
#define LEFT_MOTOR_PWM 12   // GPIO12 is a boot-strapping pin on the ESP32
#define RIGHT_MOTOR_CTRL 13
#define RIGHT_MOTOR_PWM 26

#define LEFT_ENCODER_PIN 35
#define RIGHT_ENCODER_PIN 34

#define BUTTON_PIN 27
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// prints to USB serial and to the phone if one is connected
#define TELE(...) do { Serial.printf(__VA_ARGS__); if (SerialBT.hasClient()) SerialBT.printf(__VA_ARGS__); } while (0)

// ---------------- MOTOR DRIVER SETUP ----------------
// The rest of the code uses "logical" PWM: 0 = stopped, bigger = faster.
// motors() and stop() convert it for your driver:
//   0 = DIR + PWM, normal    (duty up = faster, both directions)
//   1 = DIR + PWM, inverted  (duty 0 = fastest, both directions)
//   2 = IN1/IN2 style        (forward inverted, reverse normal)
#define DRIVER_MODE 2

// Set to 1 to run ONLY a bench test (wheels off the ground) that prints which
// ctrl/pwm combos each motor runs at. Use it if the mouse still races.
#define MOTOR_TEST 0

// ---------------- SPEED LIMITS (logical PWM, 0-255) ----------------
#define MAX_PWM 50          // hard cap on anything reaching the motors
#define BASE_SPEED 30       // cruise PWM. Raise slowly, lower if too fast
#define TURN_MAX_PWM 40
#define STEER_MAX_PWM 15
#define USE_SPEED_PID 0     // 0 = open loop cruise, 1 = add encoder speed PID
#define TARGET_SPEED 100    // mm/s, only used if USE_SPEED_PID is 1
#define LEFT_TRIM 0         // fixed offset to fix drift
#define RIGHT_TRIM 0

// ---------------- CONSTANTS (all lengths in mm, angles in degrees) ----------------
#define PULSES_PER_REVOLUTION 600
#define MM_PER_TICK ((32.0f * PI) / PULSES_PER_REVOLUTION)
#define CELL_LENGTH_MM 180.0f          // check against your maze

#define STOPPED_THRESHOLD 100000       // us without a pulse = wheel stopped
#define ACCEPTABLE_HEADING_ERROR 1.0f  // degrees

// Wall detection (mm from sensor). Placeholders, tune on the real maze
#define FRONT_WALL_THRESHOLD 120
#define SIDE_WALL_THRESHOLD 120

#define THRESHOLD_CELLCENTRE 0.1f      // in cells

#define MAZE_DIMENSION 8

// ---------------- OBJECTS ----------------
MPU6050 gyro;
SFEVL53L1X sensorFront, sensorLeft, sensorRight;

PIDStuff turn_pid    = {0.005f, 0.0f, 0.0f, 0, 0};
PIDStuff forward_pid = {0.2f, 0.1f, 0.0f, 0, 0};
PIDStuff wall_pid    = {1.0f, 0.0f, 0.0f, 0, 0};

State current_state = StateForward;

// ---------------- VARIABLES ----------------
unsigned long last_gyro_time = 0;
float gyro_z_offset = 0;
float target_heading = 0;   // degrees, anticlockwise positive
float current_yaw_rate = 0.0f;
float current_heading = 0.0f;

float loop_dt = 0.0f;       // one dt per loop, shared by all controllers

uint8_t weight[MAZE_DIMENSION][MAZE_DIMENSION];
walls wall_location[MAZE_DIMENSION][MAZE_DIMENSION];

directions mouse_direction = NORTH;
float mouse_x = 0;          // in cells, integers are cell centres
float mouse_y = 0;
int last_mot_x = -1;
int last_mot_y = -1;

uint8_t center_low = (MAZE_DIMENSION / 2) - 1;
uint8_t center_high = MAZE_DIMENSION / 2;

const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};

float dist_front = 9999, dist_left = 9999, dist_right = 9999;
float wall_distance = 59.5f;   // side reading when centred, set in setup()

// ---------------- LOGGING ----------------
void log_msg(const char* text) {
  Serial.println(text);
}

// ---------------- QUEUE ----------------
#define QUEUE_SIZE 256
Cell queue[QUEUE_SIZE];

void enqueue(Cell c, int* tail) { queue[(*tail)++ % QUEUE_SIZE] = c; }
Cell dequeue(int* head) { return queue[(*head)++ % QUEUE_SIZE]; }

bool wall_in_direction(walls w, int d) {
  switch (d) {
    case 0: return w.north;
    case 1: return w.east;
    case 2: return w.south;
    case 3: return w.west;
  }
  return false;
}

// ---------------- GYRO ----------------
void calibrate_gyro() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += gyro.getRotationZ();
    delay(2);
  }
  gyro_z_offset = (float)sum / 500.0f;
}

// anticlockwise is positive
void update_gyro() {
  unsigned long now = micros();
  float dt = (now - last_gyro_time) / 1e6f;
  last_gyro_time = now;

  float yaw = ((float)gyro.getRotationZ() - gyro_z_offset) / 131.0f;  // deg/s
  if (fabsf(yaw) < 0.5f) yaw = 0;

  current_yaw_rate = yaw;
  current_heading += yaw * dt;
}

// ---------------- ENCODERS ----------------
volatile int left_pulses = 0;
volatile unsigned long last_left_pulse_time = 0;
volatile unsigned long left_pulse_time = 0;
volatile int right_pulses = 0;
volatile unsigned long last_right_pulse_time = 0;
volatile unsigned long right_pulse_time = 0;

void IRAM_ATTR left_encoder_addition() {
  unsigned long present = micros();
  left_pulse_time = present - last_left_pulse_time;
  last_left_pulse_time = present;
  left_pulses++;
}

void IRAM_ATTR right_encoder_addition() {
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

  if (time_since_last_pulse > STOPPED_THRESHOLD) return 0.0f;
  if (interval_snapshot == 0) return NAN;
  return MM_PER_TICK / (interval_snapshot / 1e6f);
}

void update_speeds() {
  float l = wheel_speed(left_pulse_time, last_left_pulse_time);
  float r = wheel_speed(right_pulse_time, last_right_pulse_time);
  if (!isnan(l)) left_speed = l;
  if (!isnan(r)) right_speed = r;
}

// ---------------- PID ----------------
void reset_pid(PIDStuff &pid) {
  pid.integral = 0;
  pid.last_error = 0;
}

float update_PID(PIDStuff &pid, float target, float actual, float dt) {
  if (dt <= 0.0001f) return 0;

  float error = target - actual;
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -100.0f, 100.0f);

  float derivative = (error - pid.last_error) / dt;
  pid.last_error = error;

  return pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
}

// ---------------- DISTANCE SENSORS ----------------
// raw sensor value -> calibrated mm
float convert_mm(int raw) {
  return (raw + 52.143f) / 1.55f;
}

bool poll_sensor(SFEVL53L1X &s, float &out) {
  if (!s.checkForDataReady()) return false;
  out = convert_mm(s.getDistance());
  s.clearInterrupt();
  return true;
}

// non-blocking, call every loop
void poll_sensors() {
  poll_sensor(sensorFront, dist_front);
  poll_sensor(sensorLeft, dist_left);
  poll_sensor(sensorRight, dist_right);
}

// positive = too far from left wall (drifting right). Flip the sign of the
// steering in forward_drive if it steers the wrong way on the real robot.
float wall_error() {
  bool l = dist_left < SIDE_WALL_THRESHOLD;
  bool r = dist_right < SIDE_WALL_THRESHOLD;
  if (l && r) return dist_left - dist_right;
  if (l) return dist_left - wall_distance;
  if (r) return wall_distance - dist_right;
  return 0;
}

// ---------------- MOTORS ----------------
// pwm is logical: sign = direction, magnitude = speed (0 = stopped)
void motors(int ctrl_pin, int pwm_pin, int pwm) {
  int duty = constrain(abs(pwm), 0, MAX_PWM);
  bool fwd = pwm >= 0;
  digitalWrite(ctrl_pin, fwd ? HIGH : LOW);
#if DRIVER_MODE == 0
  analogWrite(pwm_pin, duty);
#elif DRIVER_MODE == 1
  analogWrite(pwm_pin, 255 - duty);
#else
  analogWrite(pwm_pin, fwd ? 255 - duty : duty);
#endif
}

void stop() {
#if DRIVER_MODE == 1
  analogWrite(LEFT_MOTOR_PWM, 255);   // inverted: 255 = stopped
  analogWrite(RIGHT_MOTOR_PWM, 255);
#else
  digitalWrite(LEFT_MOTOR_CTRL, LOW);
  digitalWrite(RIGHT_MOTOR_CTRL, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
#endif
}

void forward_drive(float dt) {
  float base = BASE_SPEED;

#if USE_SPEED_PID
  float avg_speed = (left_speed + right_speed) / 2.0f;
  base += update_PID(forward_pid, TARGET_SPEED, avg_speed, dt);
#endif
  base = constrain(base, 0.0f, (float)MAX_PWM);

  float steer = constrain(update_PID(wall_pid, wall_error(), 0, dt), -STEER_MAX_PWM, STEER_MAX_PWM);

  // forward only: never let steering reverse a wheel while cruising.
  // positive steer = turn anticlockwise (left)
  int left_out  = constrain((int)(base - steer) + LEFT_TRIM, 0, MAX_PWM);
  int right_out = constrain((int)(base + steer) + RIGHT_TRIM, 0, MAX_PWM);

  motors(LEFT_MOTOR_CTRL, LEFT_MOTOR_PWM, left_out);
  motors(RIGHT_MOTOR_CTRL, RIGHT_MOTOR_PWM, right_out);
}

// pivot on the spot using heading error (gyro is anticlockwise positive)
void turn_drive(float dt) {
  float out = update_PID(turn_pid, target_heading, current_heading, dt);
  out = constrain(out, -TURN_MAX_PWM, TURN_MAX_PWM);

  motors(LEFT_MOTOR_CTRL, LEFT_MOTOR_PWM, (int)(-out));
  motors(RIGHT_MOTOR_CTRL, RIGHT_MOTOR_PWM, (int)(out));
}

unsigned long last_time = 0;
float time_passed() {
  unsigned long now = micros();
  float dt = (now - last_time) / 1e6f;
  last_time = now;
  return dt;
}

// bench test: wheels OFF the ground. Prints each case so you can note which are fast.
void motor_test() {
  const int ctrl_pins[2] = {LEFT_MOTOR_CTRL, RIGHT_MOTOR_CTRL};
  const int pwm_pins[2] = {LEFT_MOTOR_PWM, RIGHT_MOTOR_PWM};
  const char* names[2] = {"LEFT", "RIGHT"};

  for (int m = 0; m < 2; m++) {
    for (int c = 0; c < 2; c++) {
      for (int d = 0; d < 2; d++) {
        int ctrl = c == 0 ? HIGH : LOW;
        int duty = d == 0 ? 10 : 245;
        Serial.printf("%s ctrl=%s pwm=%d\n", names[m], ctrl == HIGH ? "HIGH" : "LOW", duty);
        digitalWrite(ctrl_pins[m], ctrl);
        analogWrite(pwm_pins[m], duty);
        delay(2500);
        stop();
        delay(500);
      }
    }
  }
  Serial.println("Test done");
  while (true) delay(1000);
}

// ---------------- MAZE LOGIC ----------------
void set_wall(int x, int y, int d) {
  switch (d) {
    case NORTH:
      wall_location[x][y].north = true;
      if (y < MAZE_DIMENSION - 1) wall_location[x][y + 1].south = true;
      break;
    case EAST:
      wall_location[x][y].east = true;
      if (x < MAZE_DIMENSION - 1) wall_location[x + 1][y].west = true;
      break;
    case SOUTH:
      wall_location[x][y].south = true;
      if (y > 0) wall_location[x][y - 1].north = true;
      break;
    case WEST:
      wall_location[x][y].west = true;
      if (x > 0) wall_location[x - 1][y].east = true;
      break;
  }
}

void update_walls() {
  int x = (int)lroundf(mouse_x);
  int y = (int)lroundf(mouse_y);
  if (x < 0 || x >= MAZE_DIMENSION || y < 0 || y >= MAZE_DIMENSION) return;

  if (dist_front < FRONT_WALL_THRESHOLD) set_wall(x, y, mouse_direction);
  if (dist_left  < SIDE_WALL_THRESHOLD)  set_wall(x, y, (mouse_direction + 3) % 4);
  if (dist_right < SIDE_WALL_THRESHOLD)  set_wall(x, y, (mouse_direction + 1) % 4);
}

void flood_fill() {
  for (int i = 0; i < MAZE_DIMENSION; i++)
    for (int j = 0; j < MAZE_DIMENSION; j++)
      weight[i][j] = 255;

  int head = 0, tail = 0;

  for (uint8_t i = center_low; i <= center_high; i++) {
    for (uint8_t j = center_low; j <= center_high; j++) {
      weight[i][j] = 0;
      enqueue(Cell{(int)i, (int)j}, &tail);
    }
  }

  while (head != tail) {
    Cell c = dequeue(&head);
    uint8_t current_weight = weight[c.x][c.y];

    for (int d = 0; d < 4; d++) {
      int nx = c.x + DX[d];
      int ny = c.y + DY[d];
      if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
        if (!wall_in_direction(wall_location[c.x][c.y], d)) {
          if (current_weight < 255 && weight[nx][ny] > current_weight + 1) {
            weight[nx][ny] = current_weight + 1;
            enqueue(Cell{nx, ny}, &tail);
          }
        }
      }
    }
  }
}

// ---------------- POSITION ----------------
long last_left_pulses = 0;
long last_right_pulses = 0;

void dead_reckoning(State state, float distance) {
  if (state == StateUTurn || state == StateTurnLeft || state == StateTurnRight) return;

  if (mouse_direction == NORTH)      mouse_y += distance;
  else if (mouse_direction == EAST)  mouse_x += distance;
  else if (mouse_direction == SOUTH) mouse_y -= distance;
  else if (mouse_direction == WEST)  mouse_x -= distance;
}

void update_position() {
  noInterrupts();
  int left_snapshot = left_pulses;
  int right_snapshot = right_pulses;
  interrupts();

  int left_delta = left_snapshot - last_left_pulses;
  int right_delta = right_snapshot - last_right_pulses;
  last_left_pulses = left_snapshot;
  last_right_pulses = right_snapshot;

  float pulses = (left_delta + right_delta) / 2.0f;
  float distance_cells = (pulses * MM_PER_TICK) / CELL_LENGTH_MM;
  dead_reckoning(current_state, distance_cells);
}

void drive() {
  update_position();
  if (current_state == StateTurnLeft || current_state == StateTurnRight || current_state == StateUTurn) {
    turn_drive(loop_dt);
  } else {
    forward_drive(loop_dt);
  }
}

bool turn_done() {
  return fabsf(target_heading - current_heading) < ACCEPTABLE_HEADING_ERROR;
}

// ---------------- EVENTS ----------------
Event update_event() {
  int cx = (int)lroundf(mouse_x);
  int cy = (int)lroundf(mouse_y);

  bool turning = (current_state == StateTurnLeft || current_state == StateTurnRight || current_state == StateUTurn);

  if (turning) {
    if (turn_done()) {
      current_heading = target_heading;  // snap, removes accumulated drift
      return EventDoneTurning;
    }
    return EventNone;
  }

  bool near_centre = fabsf(mouse_x - cx) < THRESHOLD_CELLCENTRE &&
                     fabsf(mouse_y - cy) < THRESHOLD_CELLCENTRE;
  bool new_cell = (cx != last_mot_x || cy != last_mot_y);

  if (near_centre && new_cell) {
    last_mot_x = cx;
    last_mot_y = cy;

    if ((cx == center_low || cx == center_high) &&
        (cy == center_low || cy == center_high)) {
      return EventFinish;
    }
    return EventMomentOfTruth;
  }
  return EventNone;
}

// ---------------- DECISIONS ----------------
State find_best_step() {
  int min_val = 255;
  int best_dir = -1;

  int cx = (int)lroundf(mouse_x);
  int cy = (int)lroundf(mouse_y);

  for (int d = 0; d < 4; d++) {
    int nx = cx + DX[d];
    int ny = cy + DY[d];
    if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
      if (!wall_in_direction(wall_location[cx][cy], d)) {
        if (weight[nx][ny] < min_val || (weight[nx][ny] == min_val && d == mouse_direction)) {
          min_val = weight[nx][ny];
          best_dir = d;
        }
      }
    }
  }

  if (best_dir == -1) return StateFinished;  // boxed in, stop rather than drive into a wall

  if (best_dir == mouse_direction) return StateForward;

  if (best_dir == (mouse_direction + 1) % 4) {
    mouse_direction = (directions)((mouse_direction + 1) % 4);
    return StateTurnRight;
  }
  if (best_dir == (mouse_direction + 3) % 4) {
    mouse_direction = (directions)((mouse_direction + 3) % 4);
    return StateTurnLeft;
  }
  mouse_direction = (directions)((mouse_direction + 2) % 4);
  return StateUTurn;
}

State transition(State state, Event event) {
  switch (state) {
    case StateForward:
      if (event == EventFinish) return StateFinished;
      if (event == EventMomentOfTruth) return StateMOT;
      drive();
      break;

    case StateFinished:
      stop();
      break;

    case StateMOT: {
      stop();
      // re-anchor position to the cell centre to cancel dead reckoning drift
      mouse_x = lroundf(mouse_x);
      mouse_y = lroundf(mouse_y);

      update_walls();
      flood_fill();
      State next_step = find_best_step();

      if (next_step == StateTurnLeft)  target_heading += 90;
      if (next_step == StateTurnRight) target_heading -= 90;
      if (next_step == StateUTurn)     target_heading += 180;
      return next_step;
    }

    case StateTurnLeft:
    case StateTurnRight:
    case StateUTurn:
      if (event == EventDoneTurning) return StateForward;
      drive();
      break;
  }
  return state;
}

// ---------------- SETUP / LOOP ----------------
void wait_for_button() {
  Serial.println("Press the button to start");
  while (digitalRead(BUTTON_PIN) == LOW)  delay(10);  // already held: wait for release
  while (digitalRead(BUTTON_PIN) == HIGH) delay(10);  // wait for press (active low)
  delay(50);                                           // debounce
  while (digitalRead(BUTTON_PIN) == LOW)  delay(10);  // wait for release
}

void setup() {
  // motor pins first, parked at "off" straight away
  pinMode(LEFT_MOTOR_CTRL, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_CTRL, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
#if DRIVER_MODE == 1
  digitalWrite(LEFT_MOTOR_PWM, HIGH);
  digitalWrite(RIGHT_MOTOR_PWM, HIGH);
#else
  digitalWrite(LEFT_MOTOR_PWM, LOW);
  digitalWrite(RIGHT_MOTOR_PWM, LOW);
#endif
  stop();

  Serial.begin(115200);
  SerialBT.begin("micromouse");

#if MOTOR_TEST
  delay(2000);
  motor_test();   // never returns
#endif

  Wire.begin();
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // button connects pin to GND

  gyro.initialize();

  // sensors: hold all in reset, then bring up one at a time and move its address
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  if (sensorFront.begin() != 0) { Serial.println("Front sensor failed"); while (true) delay(1000); }
  sensorFront.setI2CAddress(FRONT_ADDRESS);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (sensorLeft.begin() != 0) { Serial.println("Left sensor failed"); while (true) delay(1000); }
  sensorLeft.setI2CAddress(LEFT_ADDRESS);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (sensorRight.begin() != 0) { Serial.println("Right sensor failed"); while (true) delay(1000); }
  sensorRight.setI2CAddress(RIGHT_ADDRESS);

  // start continuous ranging once
  sensorFront.startRanging();
  sensorLeft.startRanging();
  sensorRight.startRanging();

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), left_encoder_addition, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), right_encoder_addition, RISING);

  // maze arrays
  for (uint8_t i = 0; i < MAZE_DIMENSION; i++) {
    for (uint8_t j = 0; j < MAZE_DIMENSION; j++) {
      weight[i][j] = 255;
      wall_location[i][j] = {false, false, false, false};
    }
  }
  weight[center_low][center_low]   = 0;
  weight[center_low][center_high]  = 0;
  weight[center_high][center_low]  = 0;
  weight[center_high][center_high] = 0;

  wait_for_button();

  delay(1000);       // hands off the mouse while the gyro calibrates
  calibrate_gyro();

  // get a first reading from every sensor before driving
  unsigned long t0 = millis();
  bool f = false, l = false, r = false;
  while (!(f && l && r) && millis() - t0 < 1000) {
    f = f || poll_sensor(sensorFront, dist_front);
    l = l || poll_sensor(sensorLeft, dist_left);
    r = r || poll_sensor(sensorRight, dist_right);
  }

  // auto-calibrate centred side distance (mouse must start centred with walls both sides)
  if (dist_left < SIDE_WALL_THRESHOLD && dist_right < SIDE_WALL_THRESHOLD) {
    wall_distance = (dist_left + dist_right) / 2.0f;
  }
  Serial.printf("wall_distance=%.1f\n", wall_distance);

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

  log_msg("Running...");
}

void loop() {
  loop_dt = time_passed();
  update_gyro();
  poll_sensors();
  update_speeds();

  State next = transition(current_state, update_event());
  if (next != current_state) {
    reset_pid(turn_pid);
    reset_pid(forward_pid);
    reset_pid(wall_pid);
  }
  current_state = next;

  static unsigned long t = 0;
  if (millis() - t > 200) {
    t = millis();
    TELE("st=%d x=%.2f y=%.2f hdg=%.1f spdL=%.0f spdR=%.0f wallErr=%.1f L=%.0f R=%.0f F=%.0f\n",
     current_state, mouse_x, mouse_y, current_heading,
     left_speed, right_speed, wall_error(),
     dist_left, dist_right, dist_front);
  }
}
