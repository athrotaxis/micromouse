/*
 * =============================================================================
 *  FLOOD FILL MAZE SOLVER  –  Arduino Edition
 *  MicroMouse Competition (IEEE-style, 16×16 maze)
 *
 *  Hardware: Arduino Mega 2560  ← REQUIRED for 16×16 (needs ~8 KB SRAM)
 *  Robot:    Differential drive – two rear motors + front castor wheel
 *
 *  Flowchart implementation
 *    LEFT  LOOP – Check Weight → At Finish? → Highest Weight Tile → Move/Turn
 *    RIGHT LOOP – Sense Walls  → Front/Right/Left → Add Wall Info
 *                             → Determine Weight of Squares
 *
 * -----------------------------------------------------------------------------
 *  Maze specification (from project document)
 *    • 16 × 16 grid of 18 cm × 18 cm unit squares
 *    • Walls: 5 cm high, 1.2 cm thick (±5% tolerance)
 *    • Start: corner cell (0,0), bounded on THREE sides at power-on
 *    • Goal:  2 × 2 centre zone – cells (7,7) (7,8) (8,7) (8,8)
 *             Only ONE gateway into the goal zone exists
 *
 *  Multi-run strategy (competition rules §3.3.7)
 *    Run 1  – Explore and map the maze; stop when any goal cell is entered.
 *    Return – Navigate back to (0,0) autonomously using the wall map already
 *             built (flood-fill re-seeded from start).
 *    Run 2+ – Speed run on the known shortest path; official time recorded.
 *             Repeat until the 10-minute window closes.
 *
 *  Note: the 30-second penalty for operator-returning the robot is IGNORED
 *        per project instructions.
 *
 * -----------------------------------------------------------------------------
 *  Coordinate system
 *    (0,0) = start corner (bottom-left)
 *    X increases East, Y increases North
 *
 *  Heading enum    NORTH=0  EAST=1  SOUTH=2  WEST=3
 *
 *  Wall bitmask per cell    bit0=N  bit1=E  bit2=S  bit3=W
 *
 *  SRAM usage on Mega 2560 (8 KB available)
 *    walls[][]  = 256 B
 *    weight[][] = 256 B
 *    BFS queue  = 512 B (stack-allocated, temporary)
 *    Total static maze data ≈ 512 B  →  comfortable on Mega
 * =============================================================================
 */

#include <Arduino.h>

/* =============================================================================
 *  MAZE DIMENSIONS
 * ============================================================================= */
#define MAZE_W  16
#define MAZE_H  16

/* =============================================================================
 *  GOAL ZONE  –  all four centre cells count as the finish
 * ============================================================================= */
#define N_GOALS  4
static const int8_t GOAL_X[N_GOALS] = { 7, 7, 8, 8 };
static const int8_t GOAL_Y[N_GOALS] = { 7, 8, 7, 8 };

/* =============================================================================
 *  START CELL
 * ============================================================================= */
#define START_X  0
#define START_Y  0

/* =============================================================================
 *  FLOOD-FILL SENTINEL
 * ============================================================================= */
#define FLOOD_INF  255

/* =============================================================================
 *  WALL BIT-MASKS
 * ============================================================================= */
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08

/* =============================================================================
 *  PIN DEFINITIONS  –  adjust to match your wiring
 *
 *  Motor driver: L298N (or equivalent H-bridge)
 *    Each channel: one PWM pin (speed) + two direction pins
 *
 *  Distance sensors: HC-SR04 ultrasonic (TRIG + ECHO)
 *    Three sensors: front, right, left
 * ============================================================================= */

/* Left motor  (L298N channel A) */
#define PIN_MOTOR_L_PWM   2    /* ENA – must be a PWM-capable pin on Mega     */
#define PIN_MOTOR_L_IN1   22
#define PIN_MOTOR_L_IN2   23

/* Right motor  (L298N channel B) */
#define PIN_MOTOR_R_PWM   3    /* ENB – must be a PWM-capable pin on Mega     */
#define PIN_MOTOR_R_IN1   24
#define PIN_MOTOR_R_IN2   25

/* HC-SR04 – Front sensor */
#define PIN_SONAR_F_TRIG  26
#define PIN_SONAR_F_ECHO  27

/* HC-SR04 – Right sensor */
#define PIN_SONAR_R_TRIG  28
#define PIN_SONAR_R_ECHO  29

/* HC-SR04 – Left sensor */
#define PIN_SONAR_L_TRIG  30
#define PIN_SONAR_L_ECHO  31

/* =============================================================================
 *  TUNABLE CONSTANTS  –  calibrate on your robot before competition day
 * ============================================================================= */

/*
 *  WALL_DIST_CM
 *    Distance in cm below which a sensor reading declares "wall present."
 *    Typical value: hold an object at the real wall distance and print
 *    sonarCm() to Serial; set threshold a few cm above that reading.
 */
#define WALL_DIST_CM    12

/*
 *  CELL_TRAVEL_MS
 *    Time (ms) for drive_forward() to cover exactly one 18 cm cell.
 *    Calibrate: call drive_forward(1.0f) once, measure distance, adjust.
 */
#define CELL_TRAVEL_MS  550

/*
 *  TURN_90_MS
 *    Time (ms) for a 90° in-place spin at TURN_SPEED.
 *    Calibrate: call turn_right() in isolation, measure angle, adjust.
 */
#define TURN_90_MS      320

/* Motor PWM values 0–255 */
#define MOTOR_SPEED     190    /* straight-line cruising speed                 */
#define TURN_SPEED      155    /* spin speed during turns                      */

/*
 *  SPEED_RUN_FACTOR
 *    Multiplier applied to MOTOR_SPEED and (inversely) CELL_TRAVEL_MS
 *    during speed runs (Run 2+).  Start at 1.0 and increase in 0.1 steps.
 *    Caution: too high will cause skidding on the non-gloss painted floor.
 */
#define SPEED_RUN_FACTOR  1.0f

/* =============================================================================
 *  DATA TYPES
 * ============================================================================= */

typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 } Heading;

/*
 *  Phase drives the multi-run state machine.
 *
 *  PHASE_EXPLORE  – Run 1: discover walls en route to the goal.
 *  PHASE_RETURN   – Autonomously return to start after first goal contact.
 *  PHASE_SPEEDRUN – Timed runs 2, 3, … on the fully-known map.
 */
typedef enum {
    PHASE_EXPLORE,
    PHASE_RETURN,
    PHASE_SPEEDRUN
} Phase;

typedef struct {
    int8_t  x, y;
    Heading heading;
} Robot;

/* =============================================================================
 *  GLOBAL STATE
 * ============================================================================= */

static uint8_t walls [MAZE_H][MAZE_W];
static uint8_t weight[MAZE_H][MAZE_W];

static Robot robot;
static Phase phase = PHASE_EXPLORE;

/* =============================================================================
 *  SECTION 1 – MOTOR DRIVERS
 * ============================================================================= */

static void motorLeft(int speed)
{
    if (speed >= 0) {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite (PIN_MOTOR_L_PWM, speed);
    } else {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        analogWrite (PIN_MOTOR_L_PWM, -speed);
    }
}

static void motorRight(int speed)
{
    if (speed >= 0) {
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        analogWrite (PIN_MOTOR_R_PWM, speed);
    } else {
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        analogWrite (PIN_MOTOR_R_PWM, -speed);
    }
}

static void motorsStop(void)
{
    analogWrite(PIN_MOTOR_L_PWM, 0);
    analogWrite(PIN_MOTOR_R_PWM, 0);
    digitalWrite(PIN_MOTOR_L_IN1, LOW); digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW); digitalWrite(PIN_MOTOR_R_IN2, LOW);
}

/* =============================================================================
 *  SECTION 2 – ULTRASONIC DISTANCE SENSOR
 * ============================================================================= */

static uint16_t sonarCm(uint8_t trig, uint8_t echo)
{
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long dur = pulseIn(echo, HIGH, 30000UL);
    return (dur == 0) ? 400 : (uint16_t)(dur / 58UL);
}

/* =============================================================================
 *  SECTION 3 – MOVEMENT  (flowchart green action boxes)
 *
 *  speedFactor scales MOTOR_SPEED and inversely scales CELL_TRAVEL_MS so the
 *  robot still covers exactly one cell regardless of the speed setting.
 * ============================================================================= */

static void drive_forward(float speedFactor)
{
    int spd  = constrain((int)(MOTOR_SPEED * speedFactor), 0,  255);
    int trav = constrain((int)(CELL_TRAVEL_MS / speedFactor), 50, 5000);
    Serial.println(F("[HW] Drive forward"));
    motorLeft(spd); motorRight(spd);
    delay(trav);
    motorsStop();
    delay(40);
}

static void turn_right(void)
{
    Serial.println(F("[HW] Turn right 90deg"));
    motorLeft( TURN_SPEED); motorRight(-TURN_SPEED);
    delay(TURN_90_MS);
    motorsStop(); delay(40);
}

static void turn_left(void)
{
    Serial.println(F("[HW] Turn left 90deg"));
    motorLeft(-TURN_SPEED); motorRight( TURN_SPEED);
    delay(TURN_90_MS);
    motorsStop(); delay(40);
}

static void turn_180(void)
{
    Serial.println(F("[HW] Turn 180deg"));
    motorLeft( TURN_SPEED); motorRight(-TURN_SPEED);
    delay(TURN_90_MS * 2);
    motorsStop(); delay(40);
}

/* =============================================================================
 *  SECTION 4 – WALL SENSING  (right-loop parallelogram: "Sense for walls")
 * ============================================================================= */

static void sense_walls(bool *front, bool *right, bool *left)
{
    uint16_t df = sonarCm(PIN_SONAR_F_TRIG, PIN_SONAR_F_ECHO);
    uint16_t dr = sonarCm(PIN_SONAR_R_TRIG, PIN_SONAR_R_ECHO);
    uint16_t dl = sonarCm(PIN_SONAR_L_TRIG, PIN_SONAR_L_ECHO);
    *front = (df < WALL_DIST_CM);
    *right = (dr < WALL_DIST_CM);
    *left  = (dl < WALL_DIST_CM);
    Serial.print(F("[SENSE] F=")); Serial.print(df);
    Serial.print(F("cm R="));     Serial.print(dr);
    Serial.print(F("cm L="));     Serial.print(dl);
    Serial.println(F("cm"));
}

/* =============================================================================
 *  SECTION 5 – WALL DATABASE
 * ============================================================================= */

static Heading absoluteHeading(Heading h, int8_t offset)
{
    return (Heading)(((int8_t)h + offset + 4) % 4);
}

static void add_wall(int8_t x, int8_t y, Heading dir)
{
    if (x < 0 || x >= MAZE_W || y < 0 || y >= MAZE_H) return;
    switch (dir) {
        case NORTH:
            walls[y][x] |= WALL_N;
            if (y + 1 < MAZE_H) walls[y+1][x] |= WALL_S;
            break;
        case EAST:
            walls[y][x] |= WALL_E;
            if (x + 1 < MAZE_W) walls[y][x+1] |= WALL_W;
            break;
        case SOUTH:
            walls[y][x] |= WALL_S;
            if (y - 1 >= 0) walls[y-1][x] |= WALL_N;
            break;
        case WEST:
            walls[y][x] |= WALL_W;
            if (x - 1 >= 0) walls[y][x-1] |= WALL_E;
            break;
    }
}

/* =============================================================================
 *  SECTION 6 – FLOOD FILL  ("Determine weight of squares")
 *
 *  flood_fill_to_goals()  – BFS seeded from all four goal cells simultaneously.
 *                           Used during PHASE_EXPLORE and PHASE_SPEEDRUN.
 *
 *  flood_fill_to_start()  – BFS seeded from (START_X, START_Y).
 *                           Used during PHASE_RETURN.
 *
 *  Multi-goal seeding means every cell automatically gets its distance to the
 *  nearest goal cell, which is correct for the 2×2 centre zone.
 * ============================================================================= */

static void flood_fill_to_goals(void)
{
    uint8_t  queue[MAZE_W * MAZE_H * 2];
    uint16_t head = 0, tail = 0;

    memset(weight, FLOOD_INF, sizeof(weight));

    for (uint8_t i = 0; i < N_GOALS; i++) {
        int8_t gx = GOAL_X[i], gy = GOAL_Y[i];
        if (weight[gy][gx] == FLOOD_INF) {
            weight[gy][gx] = 0;
            queue[tail++]  = (uint8_t)gx;
            queue[tail++]  = (uint8_t)gy;
        }
    }

    const int8_t  dx[]     = {  0, 1,  0, -1 };
    const int8_t  dy[]     = {  1, 0, -1,  0 };
    const uint8_t nbWall[] = { WALL_N, WALL_E, WALL_S, WALL_W };

    while (head < tail) {
        uint8_t cx = queue[head++], cy = queue[head++];
        uint8_t w  = weight[cy][cx], wl = walls[cy][cx];
        for (uint8_t d = 0; d < 4; d++) {
            if (wl & nbWall[d]) continue;
            int8_t nx = (int8_t)cx + dx[d];
            int8_t ny = (int8_t)cy + dy[d];
            if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
            if (weight[ny][nx] == FLOOD_INF) {
                weight[ny][nx] = w + 1;
                queue[tail++]  = (uint8_t)nx;
                queue[tail++]  = (uint8_t)ny;
            }
        }
    }
}

static void flood_fill_to_start(void)
{
    uint8_t  queue[MAZE_W * MAZE_H * 2];
    uint16_t head = 0, tail = 0;

    memset(weight, FLOOD_INF, sizeof(weight));
    weight[START_Y][START_X] = 0;
    queue[tail++] = START_X;
    queue[tail++] = START_Y;

    const int8_t  dx[]     = {  0, 1,  0, -1 };
    const int8_t  dy[]     = {  1, 0, -1,  0 };
    const uint8_t nbWall[] = { WALL_N, WALL_E, WALL_S, WALL_W };

    while (head < tail) {
        uint8_t cx = queue[head++], cy = queue[head++];
        uint8_t w  = weight[cy][cx], wl = walls[cy][cx];
        for (uint8_t d = 0; d < 4; d++) {
            if (wl & nbWall[d]) continue;
            int8_t nx = (int8_t)cx + dx[d];
            int8_t ny = (int8_t)cy + dy[d];
            if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
            if (weight[ny][nx] == FLOOD_INF) {
                weight[ny][nx] = w + 1;
                queue[tail++]  = (uint8_t)nx;
                queue[tail++]  = (uint8_t)ny;
            }
        }
    }
}

/* =============================================================================
 *  SECTION 7 – RIGHT LOOP: sense_and_update()
 *
 *  Directly implements the right-hand side of the flowchart:
 *    Sense for walls → Front wall? → Add wall info
 *                   → Right wall? → Add wall info
 *                   → Left wall?  → Add wall info
 *                   → Determine weight of squares
 *
 *  Skipped during PHASE_SPEEDRUN (maze fully known; sensing would only slow
 *  the run and cannot reveal new walls under competition rule §3.3.3).
 * ============================================================================= */

static void sense_and_update(void)
{
    bool front_wall, right_wall, left_wall;
    sense_walls(&front_wall, &right_wall, &left_wall);

    /* Front wall? → Add wall relevant wall info */
    if (front_wall) {
        add_wall(robot.x, robot.y, absoluteHeading(robot.heading, 0));
        Serial.println(F("[WALL] Front"));
    }
    /* Right wall? → Add wall relevant wall info */
    if (right_wall) {
        add_wall(robot.x, robot.y, absoluteHeading(robot.heading, 1));
        Serial.println(F("[WALL] Right"));
    }
    /* Left wall? → Add wall relevant wall info */
    if (left_wall) {
        add_wall(robot.x, robot.y, absoluteHeading(robot.heading, 3));
        Serial.println(F("[WALL] Left"));
    }

    /* Determine weight of squares */
    if (phase == PHASE_RETURN) {
        flood_fill_to_start();
    } else {
        flood_fill_to_goals();
    }
}

/* =============================================================================
 *  SECTION 8 – LEFT LOOP HELPERS
 * ============================================================================= */

static bool is_at_goal(void)
{
    for (uint8_t i = 0; i < N_GOALS; i++)
        if (robot.x == GOAL_X[i] && robot.y == GOAL_Y[i]) return true;
    return false;
}

static bool is_at_start(void)
{
    return (robot.x == START_X && robot.y == START_Y);
}

/*
 *  best_neighbour_direction – find the open neighbour with the lowest weight.
 *
 *  "Highest weight tile" in the flowchart diamond = the neighbour that is the
 *  highest priority target = lowest numerical flood-fill value (goal = 0).
 *
 *  Returns absolute heading 0–3, or -1 if surrounded by walls.
 */
static int8_t best_neighbour_direction(void)
{
    int8_t  best_dir = -1;
    uint8_t best_w   = FLOOD_INF;

    const int8_t  dx[]     = {  0, 1,  0, -1 };
    const int8_t  dy[]     = {  1, 0, -1,  0 };
    const uint8_t nbWall[] = { WALL_N, WALL_E, WALL_S, WALL_W };

    for (uint8_t d = 0; d < 4; d++) {
        if (walls[robot.y][robot.x] & nbWall[d]) continue;
        int8_t nx = robot.x + dx[d];
        int8_t ny = robot.y + dy[d];
        if (nx < 0 || nx >= MAZE_W || ny < 0 || ny >= MAZE_H) continue;
        if (weight[ny][nx] < best_w) {
            best_w   = weight[ny][nx];
            best_dir = (int8_t)d;
        }
    }
    return best_dir;
}

/*
 *  execute_move – turn to face targetHeading then drive one cell forward.
 *
 *  Corresponds to the four green action boxes in the flowchart:
 *    Turn Right  |  Turn Left  |  180 turn  |  Travel forward
 */
static void execute_move(int8_t targetHeading, float speedFactor)
{
    int8_t delta = ((int8_t)targetHeading - (int8_t)robot.heading + 4) % 4;

    switch (delta) {
        case 0: /* already facing target – Travel forward */
            Serial.println(F("[NAV] Forward"));
            drive_forward(speedFactor);
            break;
        case 1: /* Turn Right then forward */
            Serial.println(F("[NAV] Turn Right + Forward"));
            turn_right();
            robot.heading = (Heading)targetHeading;
            drive_forward(speedFactor);
            break;
        case 2: /* 180 turn then forward */
            Serial.println(F("[NAV] 180 + Forward"));
            turn_180();
            robot.heading = (Heading)targetHeading;
            drive_forward(speedFactor);
            break;
        case 3: /* Turn Left then forward */
            Serial.println(F("[NAV] Turn Left + Forward"));
            turn_left();
            robot.heading = (Heading)targetHeading;
            drive_forward(speedFactor);
            break;
    }

    /* Update logical position */
    const int8_t dx[] = { 0, 1, 0, -1 };
    const int8_t dy[] = { 1, 0, -1, 0 };
    robot.x += dx[targetHeading];
    robot.y += dy[targetHeading];

    Serial.print(F("[POS] (")); Serial.print(robot.x);
    Serial.print(F(","));      Serial.print(robot.y);
    Serial.print(F(") h="));   Serial.println(robot.heading);
}

/* =============================================================================
 *  SECTION 9 – MAZE INITIALISATION
 *
 *  The start cell (0,0) is bounded on THREE sides (competition rule §3.2.3).
 *  The open side faces into the maze.  For a bottom-left start facing North,
 *  the South and West walls come from the outer perimeter; we add the North
 *  wall of the start cell to close the third side.
 *
 *  Adjust add_wall(START_X, START_Y, ???) if your robot enters the maze
 *  facing a different direction (e.g., EAST → add NORTH and SOUTH instead).
 * ============================================================================= */

static void init_maze(void)
{
    memset(walls,  0, sizeof(walls));
    memset(weight, 0, sizeof(weight));

    /* Outer perimeter */
    for (uint8_t x = 0; x < MAZE_W; x++) {
        walls[0][x]        |= WALL_S;
        walls[MAZE_H-1][x] |= WALL_N;
    }
    for (uint8_t y = 0; y < MAZE_H; y++) {
        walls[y][0]        |= WALL_W;
        walls[y][MAZE_W-1] |= WALL_E;
    }

    /*
     *  Start cell pre-loaded walls (three sides bounded):
     *  South and West already set above.  Add North.
     *  If your robot faces East at start, replace NORTH with SOUTH here.
     */
    add_wall(START_X, START_Y, NORTH);
}

/* =============================================================================
 *  SECTION 10 – DEBUG: print weight map over Serial Monitor
 * ============================================================================= */

static void print_weights(void)
{
    Serial.println(F("-- Weight map (R=robot  G=goal  S=start) --"));
    for (int8_t y = MAZE_H - 1; y >= 0; y--) {
        for (uint8_t x = 0; x < MAZE_W; x++) {
            bool is_robot = (x == (uint8_t)robot.x && y == robot.y);
            bool is_goal  = false;
            for (uint8_t i = 0; i < N_GOALS; i++)
                if (x == (uint8_t)GOAL_X[i] && y == GOAL_Y[i]) is_goal = true;
            bool is_start = (x == START_X && y == START_Y);

            if      (is_robot) Serial.print(F("  R"));
            else if (is_goal)  Serial.print(F("  G"));
            else if (is_start) Serial.print(F("  S"));
            else if (weight[y][x] == FLOOD_INF) Serial.print(F("  ."));
            else {
                Serial.print(weight[y][x] < 10 ? F("  ") : F(" "));
                Serial.print(weight[y][x]);
            }
        }
        Serial.println();
    }
    Serial.println();
}

/* =============================================================================
 *  ARDUINO ENTRY POINTS
 * ============================================================================= */

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("=== MicroMouse Flood Fill – 16x16 (Mega 2560) ==="));
    Serial.println(F("Goal: cells (7,7)(7,8)(8,7)(8,8)  Start: (0,0)"));

    /* Motor driver pins */
    pinMode(PIN_MOTOR_L_PWM, OUTPUT); pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT); pinMode(PIN_MOTOR_R_PWM, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT); pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    motorsStop();

    /* Ultrasonic sensor pins */
    pinMode(PIN_SONAR_F_TRIG, OUTPUT); pinMode(PIN_SONAR_F_ECHO, INPUT);
    pinMode(PIN_SONAR_R_TRIG, OUTPUT); pinMode(PIN_SONAR_R_ECHO, INPUT);
    pinMode(PIN_SONAR_L_TRIG, OUTPUT); pinMode(PIN_SONAR_L_ECHO, INPUT);

    /* Robot starting state */
    robot.x       = START_X;
    robot.y       = START_Y;
    robot.heading = NORTH;   /* change to EAST if robot faces East at start    */
    phase         = PHASE_EXPLORE;

    /* Maze: perimeter + known start-cell walls */
    init_maze();

    /* First flood fill – only perimeter and start walls known yet */
    flood_fill_to_goals();

    Serial.println(F("Waiting 3 s – position the robot then stand clear..."));
    delay(3000);
}

/*
 * =============================================================================
 *  MAIN LOOP – three-phase state machine
 *
 *  PHASE_EXPLORE  → discover the maze and reach the goal (Run 1)
 *  PHASE_RETURN   → return to start autonomously after first goal contact
 *  PHASE_SPEEDRUN → fast timed runs on the fully-known map (Run 2, 3, …)
 * =============================================================================
 */
void loop(void)
{
    switch (phase) {

        /* ════════════════════════════════════════════════════════════════
         *  PHASE_EXPLORE  –  implements BOTH flowchart loops together
         * ════════════════════════════════════════════════════════════════ */
        case PHASE_EXPLORE: {

            /* ── RIGHT LOOP: Sense for walls → update map → recompute weights */
            sense_and_update();

            /* ── LEFT LOOP: Check Weight */
            Serial.print(F("[EXPLORE] (")); Serial.print(robot.x);
            Serial.print(F(",")); Serial.print(robot.y);
            Serial.print(F(") w=")); Serial.println(weight[robot.y][robot.x]);

            /* ── At Finish? */
            if (is_at_goal()) {
                Serial.println(F("\n*** GOAL REACHED – switching to RETURN ***"));
                flood_fill_to_start();
                phase = PHASE_RETURN;
                return;
            }

            /* ── Highest weight tile → pick direction */
            int8_t target = best_neighbour_direction();
            if (target < 0) {
                motorsStop();
                Serial.println(F("[ERR] No accessible neighbour – halting"));
                while (true) { }
            }

            /* ── Execute move (Turn Right / Left / 180 / Forward) */
            execute_move(target, 1.0f);
            print_weights();
            break;
        }

        /* ════════════════════════════════════════════════════════════════
         *  PHASE_RETURN  –  autonomous return to start
         *
         *  Wall sensing continues so any missed walls get recorded, making
         *  the speed-run map as complete as possible.
         *  Per competition rule §3.3.7 the robot "may and should" make
         *  several runs without being touched by the operator.
         * ════════════════════════════════════════════════════════════════ */
        case PHASE_RETURN: {

            /* Sense walls on return trip to fill any map gaps */
            sense_and_update();   /* internally calls flood_fill_to_start() */

            Serial.print(F("[RETURN] (")); Serial.print(robot.x);
            Serial.print(F(",")); Serial.print(robot.y);
            Serial.print(F(") w=")); Serial.println(weight[robot.y][robot.x]);

            if (is_at_start()) {
                Serial.println(F("\n*** AT START – switching to SPEEDRUN ***"));
                /*
                 *  Recompute goal-directed weights one final time with the
                 *  now-complete (or near-complete) wall map.
                 *  Speed runs use this static map; no further sensing.
                 */
                flood_fill_to_goals();
                phase = PHASE_SPEEDRUN;
                delay(500);   /* brief pause before first timed run begins    */
                return;
            }

            int8_t target = best_neighbour_direction();
            if (target < 0) {
                motorsStop();
                Serial.println(F("[ERR] Stuck during return – halting"));
                while (true) { }
            }
            execute_move(target, 1.0f);
            break;
        }

        /* ════════════════════════════════════════════════════════════════
         *  PHASE_SPEEDRUN  –  timed runs (Run 2, 3, …)
         *
         *  The maze map is fully known.  No wall sensing is performed –
         *  this maximises speed and matches competition rule §3.3.3 which
         *  prohibits reprogramming but allows speed-setting changes.
         *
         *  Each loop() call executes one complete round-trip:
         *    Outbound  – start → goal   (timer runs; official time recorded)
         *    Return    – goal  → start  (not timed; no penalty per project)
         *
         *  The robot repeats until the 10-minute window expires externally.
         * ════════════════════════════════════════════════════════════════ */
        case PHASE_SPEEDRUN: {

            /* --- Outbound leg: start → goal (official timed run) -------- */
            flood_fill_to_goals();
            Serial.println(F("[SPEED] Outbound: START → GOAL"));

            while (!is_at_goal()) {
                int8_t target = best_neighbour_direction();
                if (target < 0) {
                    motorsStop();
                    Serial.println(F("[ERR] Stuck on speed run – halting"));
                    while (true) { }
                }
                execute_move(target, SPEED_RUN_FACTOR);
            }
            Serial.println(F("[SPEED] GOAL reached"));

            /* --- Return leg: goal → start (not officially timed) -------- */
            flood_fill_to_start();
            Serial.println(F("[SPEED] Return: GOAL → START"));

            while (!is_at_start()) {
                int8_t target = best_neighbour_direction();
                if (target < 0) {
                    motorsStop();
                    Serial.println(F("[ERR] Stuck on return leg – halting"));
                    while (true) { }
                }
                execute_move(target, 1.0f);   /* return at normal speed       */
            }

            Serial.println(F("[SPEED] Back at start – next run in 0.5 s"));
            delay(500);
            /* loop() called again → another outbound speed run begins */
            break;
        }
    }
}

/*
 * =============================================================================
 *  CALIBRATION GUIDE
 *
 *  1. CELL_TRAVEL_MS  (target: robot moves exactly 18 cm per call)
 *     Open Serial Monitor at 115200 baud.  Temporarily call drive_forward(1.0f)
 *     once in setup() with sense_and_update() commented out.  Measure the
 *     distance travelled with a ruler against a 18 cm floor mark.
 *
 *  2. TURN_90_MS  (target: robot rotates exactly 90°)
 *     Call turn_right() in setup().  Mark starting and ending orientations
 *     on the floor.  A protractor or set-square helps.  Adjust until exact.
 *
 *  3. WALL_DIST_CM  (target: reliably detect 5 cm walls from inside 18 cm cell)
 *     Hold a flat board at the actual wall face distance from your sensor and
 *     Serial.print(sonarCm(...)).  Set WALL_DIST_CM 2–3 cm above that reading.
 *     Typical sensor-to-wall distance inside an 18 cm cell ≈ 6–9 cm depending
 *     on robot length.  Account for the 1.2 cm wall thickness.
 *
 *  4. MOTOR_SPEED / TURN_SPEED
 *     Start low (120 / 100) on the painted plywood floor (which may be slick –
 *     see maze spec §3.2.2 warning).  Increase gradually watching for slip.
 *
 *  5. SPEED_RUN_FACTOR
 *     Begin at 1.0 (no change).  After a confirmed correct mapping run,
 *     increment by 0.1 per test.  Watch for cell overshoot at high values.
 *     A factor of 1.3–1.5 is a realistic target for a well-tuned robot.
 *
 *  6. Starting heading
 *     Change robot.heading in setup() to match physical placement.
 *     Also update add_wall() calls in init_maze() if the open exit side
 *     of the start cell differs from the default (North wall added = open East).
 *
 * =============================================================================
 *  WIRING SUMMARY  (Arduino Mega 2560)
 *
 *  L298N ENA  → Mega pin 2  (PWM)    L298N ENB  → Mega pin 3  (PWM)
 *  L298N IN1  → Mega pin 22          L298N IN3  → Mega pin 24
 *  L298N IN2  → Mega pin 23          L298N IN4  → Mega pin 25
 *
 *  HC-SR04 Front   TRIG=26  ECHO=27
 *  HC-SR04 Right   TRIG=28  ECHO=29
 *  HC-SR04 Left    TRIG=30  ECHO=31
 *
 *  Power:  L298N Vs  → 7.4 V LiPo (recommended; fits within 25×25 cm frame)
 *          L298N Vss → Mega 5 V
 *          Common GND: Mega ↔ L298N ↔ battery (–)
 *          HC-SR04:  5 V and GND from Mega
 *
 *  Max robot footprint: 25 cm × 25 cm  (competition rule §3.1.5)
 *  Min cell clearance:  18 cm − 1.2 cm (wall) = 16.8 cm clear path width
 * =============================================================================
 */
