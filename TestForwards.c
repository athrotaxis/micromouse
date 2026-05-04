

#include <stdio.h>

#include <string.h>

#include <stdint.h>

#include "API.h"

//hello

void log(char* text) {

    fprintf(stderr, "%s\n", text);

    fflush(stderr);

}

#define FLOOD_INF 255 //??

#define MAZE_DIMENSION 16


typedef enum {WALL_N = 1, WALL_E = 2, WALL_S = 4, WALL_W = 8} Heading;

/** flood_fill – BFS from the goal outward, writing distances into weight[][] */

uint8_t weight [MAZE_DIMENSION][MAZE_DIMENSION];

uint8_t walls [MAZE_DIMENSION][MAZE_DIMENSION];


static void flood_fill(void)

{

    /* Simple queue using a static array (max MAZE_W*MAZE_H entries) */

    int queue[MAZE_DIMENSION * MAZE_DIMENSION][2];

    int head = 0, tail = 0;


    /* Initialise all weights to infinity */

    memset(weight, FLOOD_INF, sizeof(weight));


    /* Seed the goal */

    weight[7][7] = 0;

    weight[7][8] = 0;

    weight[8][7] = 0;

    weight[8][8] = 0;

    int seeds[4][2] = {{7,7},{7,8},{8,7},{8,8}};

    for (int s = 0; s < 4; s++) {

        queue[tail][0] = seeds[s][0];

        queue[tail][1] = seeds[s][1];

        tail++;

    }


    /* BFS */

    while (head < tail) {

        int cx = queue[head][0];

        int cy = queue[head][1];

        head++;


        uint8_t w   = weight[cy][cx];

        uint8_t wll = walls[cy][cx];


        /* Try each neighbour */

        int dx[] = { 0,  1,  0, -1 };

        int dy[] = { 1,  0, -1,  0 };

        uint8_t nb_wall[] = { WALL_N, WALL_E, WALL_S, WALL_W };


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
    //below one is claude generated - check
    if (API_wallRight()) {
    if (mouse_direction == NORTH) {
        wall_location[x][y].east = true;
        wall_location[x+1][y].west = true;
    } else if (mouse_direction == EAST) {
        wall_location[x][y].south = true;
        wall_location[x][y-1].north = true;
    } else if (mouse_direction == SOUTH) {
        wall_location[x][y].west = true;
        wall_location[x-1][y].east = true;
    } else if (mouse_direction == WEST) {
        wall_location[x][y].north = true;
        wall_location[x][y+1].south = true;
    }
}
}
}

int main(int argc, char* argv[]) {

    flood_fill();

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

        while (API_wallFront()) {

            API_turnRight();

        }
    }

}