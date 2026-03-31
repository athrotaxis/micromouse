#include <stdio.h>
#include <string.h>

#include "API.h"
//hello
void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

#define MAZE_DIMENSION 16

int main(int argc, char* argv[]) {
    
    int weight [MAZE_DIMENSION][MAZE_DIMENSION];
    int walls [MAZE_DIMENSION][MAZE_DIMENSION];

    int queue[MAZE_DIMENSION*MAZE_DIMENSION][2];


    weight[7][7] = 0;
    weight[7][8] = 0;
    weight[8][7] = 0;
    weight[8][8] = 0;



    while (1) {
        
        
        if (!API_wallLeft()) {
            API_turnLeft();
        }
        while (API_wallFront()) {
            API_turnRight();
        }
        API_moveForward();
    }
}