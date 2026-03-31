#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "API.h"
//hello
void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

#define MAZE_DIMENSION 16

int weight [MAZE_DIMENSION][MAZE_DIMENSION];

//Make a data structure containing the possible positions of walls
typedef struct walls { 
    bool north;
    bool east;
    bool south;
    bool west;
} walls;

//Create a 2D array where each cell contains the data structure walls created above
walls wall_location[MAZE_DIMENSION][MAZE_DIMENSION];

//Store the mouse direction
typedef enum directions {NORTH, EAST, SOUTH, WEST} directions;

/*
Breadth first search flood fill algorithm
*/
void floodFill() {
    
} 

/*
Update the walls of the map
*/
void updateWalls(int x, int y, directions mouse_direction) {
    //if there is a wall in front, and mouse is facing north, must be a wall to the north of this cell, and a wall to the south of the cell 1 y unit above.
    if (API_wallFront()) {
        if(mouse_direction == NORTH) { 
            wall_location[x][y].north = true;
            wall_location[x][y+1].south = true;
            log("wall is in front");
        } else if(mouse_direction == EAST) {
            wall_location[x][y].east = true;
            wall_location[x+1][y].west = true;
            log("wall is in east");
        } else if (mouse_direction = SOUTH) {
            wall_location[x][y].south = true;
            wall_location[x][y-1].north = true;
        } else if (mouse_direction = WEST) {
            wall_location[x][y].west = true;
            wall_location[x-1][y].east = true;
        }
    }
    
    
}

int main(int argc, char* argv[]) {
    log("hello world");
    enum directions mouse_direction = NORTH;
    //Set center positions
    weight[7][7] = 0;
    weight[7][8] = 0;
    weight[8][7] = 0;
    weight[8][8] = 0;

    //initialise the 2 arrays as it's bad practice not to
    for (int i=0; i < MAZE_DIMENSION; i++) {
        for (int j=0; j < MAZE_DIMENSION; j++) {
            weight[i][j] = 999; //initialise to infinity for some reason not sure?
            wall_location[i][j].north = false;
            wall_location[i][j].east = false;
            wall_location[i][j].south = false;
            wall_location[i][j].west = false;
        }
    }
    

    while (1) {
        updateWalls(0,2,mouse_direction);
        if (!API_wallLeft()) {
            API_turnLeft();
        }
        while (API_wallFront()) {
            API_turnRight();
        }
        API_moveForward();
    }
}


#define FLOOD_INF 255 //??
typedef enum {WALL_N = 0, WALL_E = 1, WALL_S = 2, WALL_W = 3} Heading;
/** flood_fill – BFS from the goal outward, writing distances into weight[][] */
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
    queue[tail][0] = 7;
    queue[tail][1] = 7;
    tail++;

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
            if (wll & nb_wall[d]) continue;   /* wall blocks this direction */
            int nx = cx + dx[d];
            int ny = cy + dy[d];
            if (nx < 0 || nx >= MAZE_DIMENSION || ny < 0 || ny >= MAZE_DIMENSION) continue;
            if (weight[ny][nx] == FLOOD_INF) {
                weight[ny][nx] = w + 1;
                queue[tail][0] = nx;
                queue[tail][1] = ny;
                tail++;
            }
        }
    }
    
    for (int i = 0; i<MAZE_DIMENSION; i++) {
        for (int j=0; j<MAZE_DIMENSION; j++) {
             char buf[4];
            snprintf(buf, sizeof(buf), "%d", weight[i][j]);
            API_setText(i, j, buf);
        }
    }
}



