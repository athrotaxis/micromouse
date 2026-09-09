#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "API.h" //contains the functions that MMS simulator uses. will not be included in The Program.

#define MAZE_DIMENSION 16

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
int mouse_x = 0;
int mouse_y = 0;

//store cell locations (x, y coords) to iterate thru
typedef struct {
    int x, y;
} Cell;

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

/*
Calculates the weight of each cell

This calculation is done using BFS seeded from the center 4 nodes
This is *not* the same as mouse doing an exhaustive search using BFS seeded from the starting point


*/
void flood_fill()
{
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


/*
Move to the neighbour with the lowest cost
From a Python program by @alfredjoejr on Github: https://github.com/alfredjoejr/mms-python/blob/main/Main.py
Translated into C using Claude
*/
void move_best_step()
{
    int min_val = 255;
    int best_dir = -1; //forgot

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    // Check all 4 neighbours to see which is a) not blocked by a wall and b) has the lowest weight
    for (int d_check = 0; d_check < 4; d_check++) {
        int nx = mouse_x + dx[d_check];
        int ny = mouse_y + dy[d_check];
        if (nx >= 0 && nx < MAZE_DIMENSION && ny >= 0 && ny < MAZE_DIMENSION) {
            // If path is clear in memory
            if (!wall_in_direction(wall_location[mouse_x][mouse_y], d_check)) {
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

        //and then move this line to another function, so after having called this function and then making mouse turn appropriately it will go forwards
        API_moveForward();

        //change position after moving forwards
        if (mouse_direction == NORTH) mouse_y++;
        if (mouse_direction == EAST)  mouse_x++;
        if (mouse_direction == SOUTH) mouse_y--;
        if (mouse_direction == WEST)  mouse_x--;
    }
}



/*
Update the walls of the map based on the sensor reading

Its pretty obvious I wrote this with no research... claude has suggested an efficient way of doing it
probably would call this func whenever mouse in the center of the cell? and replace "API_wallFront()" with some sensing fucntion?
*/
/*


void updateWalls(int x, int y, directions mouse_direction)
{
    //if there is a wall in front, and mouse is facing north, must be a wall to the north of this cell, and a wall to the south of the cell 1 y unit above.

    if (API_wallFront()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].west = true;
            wall_location[x-1][y].east = true;
            //API_setWall(x, y, 'n');
        } else if(mouse_direction == EAST) {
            wall_location[x][y].east = true;
            wall_location[x+1][y].west = true;
            //API_setWall(x, y, 'e');
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            wall_location[x][y-1].north = true;
            //API_setWall(x, y, 's');
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            wall_location[x-1][y].east = true;
            //API_setWall(x, y, 'w');
        }
    }
    
    if (API_wallLeft()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].east = true; //wrong??
            wall_location[x-1][y].west = true;   // cell to the WEST
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
    //below one is claude generated - check
    //check if its in bounds??
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
*/
/*
void updateWalls(int x, int y, directions mouse_direction) {
if (API_wallFront()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].north = true;
            if (y < MAZE_DIMENSION - 1) wall_location[x][y+1].south = true;
        } else if (mouse_direction == EAST) {
            wall_location[x][y].east = true;
            if (x < MAZE_DIMENSION - 1) wall_location[x+1][y].west = true;
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            if (y > 0) wall_location[x][y-1].north = true;
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            if (x > 0) wall_location[x-1][y].east = true;
        }
    }

    if (API_wallLeft()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].west = true;
            if (x > 0) wall_location[x-1][y].east = true;
        } else if (mouse_direction == EAST) {
            wall_location[x][y].north = true;
            if (y < MAZE_DIMENSION - 1) wall_location[x][y+1].south = true;
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].east = true;
            if (x < MAZE_DIMENSION - 1) wall_location[x+1][y].west = true;
        } else if (mouse_direction == WEST) {
            wall_location[x][y].south = true;
            if (y > 0) wall_location[x][y-1].north = true;
        }
    }

    if (API_wallRight()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].east = true;
            if (x < MAZE_DIMENSION - 1) wall_location[x+1][y].west = true;
        } else if (mouse_direction == EAST) {
            wall_location[x][y].south = true;
            if (y > 0) wall_location[x][y-1].north = true;
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].west = true;
            if (x > 0) wall_location[x-1][y].east = true;
        } else if (mouse_direction == WEST) {
            wall_location[x][y].north = true;
            if (y < MAZE_DIMENSION - 1) wall_location[x][y+1].south = true;
        }
    }
}
*/
void updateWalls(int x, int y, directions mouse_direction) {
    if (API_wallFront()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].north = true;
            API_setWall(x, y, 'n');
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; API_setWall(x, y+1, 's'); }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].east = true;
            API_setWall(x, y, 'e');
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true; API_setWall(x+1, y, 'w'); }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].south = true;
            API_setWall(x, y, 's');
            if (y > 0) { wall_location[x][y-1].north = true; API_setWall(x, y-1, 'n'); }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].west = true;
            API_setWall(x, y, 'w');
            if (x > 0) { wall_location[x-1][y].east = true; API_setWall(x-1, y, 'e'); }
        }
    }

    if (API_wallLeft()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].west = true;
            API_setWall(x, y, 'w');
            if (x > 0) { wall_location[x-1][y].east = true; API_setWall(x-1, y, 'e'); }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].north = true;
            API_setWall(x, y, 'n');
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; API_setWall(x, y+1, 's'); }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].east = true;
            API_setWall(x, y, 'e');
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true; API_setWall(x+1, y, 'w'); }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].south = true;
            API_setWall(x, y, 's');
            if (y > 0) { wall_location[x][y-1].north = true; API_setWall(x, y-1, 'n'); }
        }
    }

    if (API_wallRight()) {
        if (mouse_direction == NORTH) {
            wall_location[x][y].east = true;
            API_setWall(x, y, 'e');
            if (x < MAZE_DIMENSION - 1) { wall_location[x+1][y].west = true; API_setWall(x+1, y, 'w'); }
        } else if (mouse_direction == EAST) {
            wall_location[x][y].south = true;
            API_setWall(x, y, 's');
            if (y > 0) { wall_location[x][y-1].north = true; API_setWall(x, y-1, 'n'); }
        } else if (mouse_direction == SOUTH) {
            wall_location[x][y].west = true;
            API_setWall(x, y, 'w');
            if (x > 0) { wall_location[x-1][y].east = true; API_setWall(x-1, y, 'e'); }
        } else if (mouse_direction == WEST) {
            wall_location[x][y].north = true;
            API_setWall(x, y, 'n');
            if (y < MAZE_DIMENSION - 1) { wall_location[x][y+1].south = true; API_setWall(x, y+1, 's'); }
        }
    }
}
int main(int argc, char* argv[])
{
    log2("Running...");
    char buf[4];  //msvc quirk, not sure if relevant for arduino

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

    for (int i = 0; i<16; i++) {
        for (int j=0; j<16; j++) {
            visited[i][j] = false; //initialise array of visited cells
        }
    }

    while (1) {
        //to add : check if in middle
        //nothing is relevant for THe Program except move_best_step()

        if (!visited[mouse_x][mouse_y]) { //colour visited cells in mms
            visited[mouse_x][mouse_y] = true; 
            API_setColor(mouse_x, mouse_y, 'c'); // cyan
        }
        updateWalls(mouse_x, mouse_y, mouse_direction);
        flood_fill();

        fprintf(stderr, "-----------------------------------------");
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

        move_best_step(); 
       
    }
}