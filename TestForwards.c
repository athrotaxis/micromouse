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

//Initialise the mouse's current direction & position
typedef enum directions {NORTH, EAST, SOUTH, WEST} directions;
enum directions mouse_direction = NORTH;
int mouse_x = 0;
int mouse_y = 0;



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
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            wall_location[x][y-1].north = true;
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            wall_location[x-1][y].east = true;
        }
    }
    
    
}

int main(int argc, char* argv[]) {
    log2("hello world");
    
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
    
    //Set center positions
    weight[7][7] = 0;
    weight[7][8] = 0;
    weight[8][7] = 0;
    weight[8][8] = 0;

    while (1) {
        //updateWalls(0,2,mouse_direction);
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
        fprintf(stderr, "Mouse direction is %d\n", mouse_direction);
        fprintf(stderr, "Mouse x coordinate is %d\n", mouse_x);
        fprintf(stderr, "Mouse y coordinate is %d\n", mouse_y);
    }
}
