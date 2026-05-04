

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






int main(int argc, char* argv[]) {

    flood_fill();

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