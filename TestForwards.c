#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "API.h"
//hello
void log2(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

#define MAZE_DIMENSION 16

uint8_t weight [MAZE_DIMENSION][MAZE_DIMENSION];

//Make a data structure containing the possible positions of walls
typedef struct walls { 
    bool north;
    bool east;
    bool south;
    bool west;
} walls;

//Initialise the mouse's current direction & position
typedef enum directions {NORTH, EAST, SOUTH, WEST} directions;
enum directions mouse_direction = NORTH;
int mouse_x = 0;
int mouse_y = 0;

//Create a 2D array where each cell contains the data structure walls created above
walls wall_location[MAZE_DIMENSION][MAZE_DIMENSION];

typedef struct
{
    uint8_t items[MAZE_DIMENSION * MAZE_DIMENSION];  // 256 slots
    uint8_t front;
    uint8_t rear;
} Queue;

//store cell location to look thru
typedef struct {
    int x, y;
} Cell;

//Make queue
#define QUEUE_SIZE 256
Cell queue[256];
int head = 0, tail = 0;

/*
Breadth first search flood fill algorithm
*/
void flood_fill()
{
    int head = 0, tail = 0;

    for (uint8_t i = 7; i < 9; i++) {
        for (uint8_t j = 7; j < 9; j++) {
            weight[i][j] = 0;
            queue[tail++] = (Cell){i, j};
        }
    }
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    while (head != tail) {
        Cell c = queue[head++ % QUEUE_SIZE];
        uint8_t current_weight = weight[c.x][c.y];

        for (int d = 0; d < 4; d++) {
            int nx = c.x + dx[d];
            int ny = c.y + dy[d];
            if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
                if (!wall_in_direction(wall_location[c.x][c.y], d)) {
                    if (current_weight < 255 && weight[nx][ny] > current_weight + 1) {
                        weight[nx][ny] = current_weight + 1;
                        queue[tail++ % QUEUE_SIZE] = (Cell){nx, ny};
                    }
                }
            }
        }
    }
}

bool wall_in_direction(walls w, int d) {
    switch(d) {
        case 0: return w.north;
        case 1: return w.east;
        case 2: return w.south;
        case 3: return w.west;
    }
    return false;
}

/*
Update the walls of the map
*/
void updateWalls(int x, int y, directions mouse_direction)
{
    //if there is a wall in front, and mouse is facing north, must be a wall to the north of this cell, and a wall to the south of the cell 1 y unit above.

    if (API_wallFront()) {
        if(mouse_direction == NORTH) { 
            wall_location[x][y].north = true;
            wall_location[x][y+1].south = true;
        } else if(mouse_direction == EAST) {
            wall_location[x][y].east = true;
            wall_location[x+1][y].west = true;
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            wall_location[x][y-1].north = true;
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            wall_location[x-1][y].east = true;
        }
    }
    
    if (API_wallLeft()) {
    if (mouse_direction == NORTH) {
        wall_location[x][y].west = true;
        wall_location[x-1][y].east = true;   // cell to the WEST
    } else if (mouse_direction == EAST) {
        wall_location[x][y].north = true;
        wall_location[x][y+1].south = true;  // cell to the NORTH
    } else if (mouse_direction == SOUTH) {
        wall_location[x][y].east = true;
        wall_location[x+1][y].west = true;   // cell to the EAST
    } else if (mouse_direction == WEST) {
        wall_location[x][y].south = true;
        wall_location[x][y-1].north = true;  // cell to the SOUTH
    }
}
}

int main(int argc, char* argv[])
{
    log2("Running...");
    
    //initialise the 2 arrays as it's bad practice not to
    for (uint8_t i=0; i < MAZE_DIMENSION; i++) {
        for (uint8_t j=0; j < MAZE_DIMENSION; j++) {
            weight[i][j] = 999; //initialise to infinity for some reason not sure?
            wall_location[i][j].north = false;
            wall_location[i][j].east = false;
            wall_location[i][j].south = false;
            wall_location[i][j].west = false;
        }
    }
    
    //Set center positions
    weight[7][7] = 0;
    weight[7][8] = 0;
    weight[8][7] = 0;
    weight[8][8] = 0;

    // Centre run
    int center_goals[4][2] = {{7,7}, {7,8}, {8,7}, {8,8}};
    flood_fill(center_goals, 4);

    // Return to start
    int start_goal[1][2] = {{0, 0}};
    flood_fill(start_goal, 1);

    //initialize queue
    initializeQueue(&queue);

    while (1) {
        updateWalls(mouse_x, mouse_y, mouse_direction);

        fprintf(stderr, "Mouse direction is %d\n", mouse_direction);
        fprintf(stderr, "Mouse x coordinate is %d\n", mouse_x);
        fprintf(stderr, "Mouse y coordinate is %d\n", mouse_y);

        if (!API_wallLeft()) {
            API_turnLeft();
           mouse_direction = (mouse_direction + 3) % 4; //rotates mouse direction by overflowing - used claude
        }
        while (API_wallFront()) {
            API_turnRight();
            mouse_direction = (mouse_direction + 1) % 4;
        }
        API_moveForward();
        if (mouse_direction == NORTH) {
            mouse_y++;
        } else if (mouse_direction == EAST) {
            mouse_x++;
        } else if (mouse_direction == SOUTH) {
            mouse_y--;
        } else {
            mouse_x--;
        }
        
        if (wall_location[mouse_x][mouse_y].north) {
            fprintf(stderr, "Wall detected to the north\n");
        }
        if (wall_location[mouse_x][mouse_y].east) {
            fprintf(stderr, "Wall detected to the east\n");
        }
        if (wall_location[mouse_x][mouse_y].south) {
            fprintf(stderr, "Wall detected to the south\n");
        }
        if (wall_location[mouse_x][mouse_y].west) {
            fprintf(stderr, "Wall detected to the west\n");
        }
    }
}