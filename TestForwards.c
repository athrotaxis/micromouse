#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "API.h"

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

//store cell location to look thru
typedef struct {
    int x, y;
} Cell;

//Make queue
#define QUEUE_SIZE 256
Cell queue[256];

bool wall_in_direction(walls w, int d) {
    switch(d) {
        case 0: return w.north;
        case 1: return w.east;
        case 2: return w.south;
        case 3: return w.west;
    }
    return false;
}

void enqueue(Cell c, int* tail) 
{ 
    queue[(*tail)++ % QUEUE_SIZE] = c; 
}
Cell dequeue(int* head)       
{ 
    return queue[(*head)++ % QUEUE_SIZE]; 
}

void flood_fill()
{
     // Reset all weights to 255 before recalculating
    for (int i = 0; i < MAZE_DIMENSION; i++) {
        for (int j = 0; j < MAZE_DIMENSION; j++) {
            weight[i][j] = 255;
        }
    }

    int head = 0, tail = 0;

    for (uint8_t i = 7; i < 9; i++) {
        for (uint8_t j = 7; j < 9; j++) {
            weight[i][j] = 0;
            queue[tail++] = (Cell){i, j};
        }
    }
    
    int dx[] = {0, 1, 0, -1}; //can't be uint8_t as its unsigned
    int dy[] = {1, 0, -1, 0};

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


/*
    Move to the neighbour with the lowest cost (preferencing driving straight)
*/
void move_best_step()
{
    int min_val = 255;
    int best_dir = -1;

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    // Check all 4 neighbours
    for (int d_check = 0; d_check < 4; d_check++) {
        int nx = mouse_x + dx[d_check];
        int ny = mouse_y + dy[d_check];
        if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
            // If path is clear in memory
            if (!wall_in_direction(wall_location[mouse_x][mouse_y], d_check)) {
                if (weight[nx][ny] < min_val || (weight[nx][ny] == min_val && d_check == mouse_direction)) { // preference going straight to prevent getting stuck
                    min_val = weight[nx][ny];
                    best_dir = d_check;
                }
            }
        }
    }

    if (best_dir != -1) {
        // Turn to face best direction
        if (best_dir == mouse_direction) {
            // already facing right way
        } else if (best_dir == (mouse_direction + 1) % 4) {
            API_turnRight();
            mouse_direction = (mouse_direction + 1) % 4;
        } else if (best_dir == (mouse_direction + 3) % 4) {
            API_turnLeft();
            mouse_direction = (mouse_direction + 3) % 4;
        } else {
            API_turnRight();
            API_turnRight();
            mouse_direction = (mouse_direction + 2) % 4;
        }

        API_moveForward();
        if (mouse_direction == NORTH) mouse_y++;
        if (mouse_direction == EAST)  mouse_x++;
        if (mouse_direction == SOUTH) mouse_y--;
        if (mouse_direction == WEST)  mouse_x--;
    }
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
    char buf[4];  //msvc quirk

    //initialise the 2 arrays as it's bad practice not to
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
    weight[7][7] = 0;
    weight[7][8] = 0;
    weight[8][7] = 0;
    weight[8][8] = 0;

    while (1) {
        flood_fill();
        fprintf(stderr, "Weight at 0,0: %d\n", weight[0][0]);
        move_best_step();
        updateWalls(mouse_x, mouse_y, mouse_direction);

        fprintf(stderr, "Mouse direction is %d\n", mouse_direction);
        fprintf(stderr, "Mouse x coordinate is %d\n", mouse_x);
        fprintf(stderr, "Mouse y coordinate is %d\n", mouse_y);

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

        for (int i = 0; i < MAZE_DIMENSION; i++) {
            for (int j = 0; j < MAZE_DIMENSION; j++) {
                sprintf(buf, "%d", weight[i][j]);
                API_setText(i, j, buf);
            }
        }
    }
}