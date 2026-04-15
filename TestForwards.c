/*
 * bfs_mouse.c
 *
 * C translation of mackorone/breadth-first-search (Main.py)
 * for use with the mackorone/mms Micromouse simulator.
 *
 * The mms simulator communicates over stdin/stdout.
 * Build:  gcc -std=c99 -o bfs_mouse bfs_mouse.c
 * Run:    ./bfs_mouse
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* ---------- tuneable constants ---------- */
#define MAX_WIDTH  16
#define MAX_HEIGHT 16
/* BFS queue needs at most W*H entries */
#define QUEUE_CAP  (MAX_WIDTH * MAX_HEIGHT)

/* ---------- Direction ---------- */
typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 } Direction;

static Direction turn_left(Direction d)  { return (Direction)((d + 3) % 4); }
static Direction turn_right(Direction d) { return (Direction)((d + 1) % 4); }
static Direction turn_around(Direction d){ return (Direction)((d + 2) % 4); }

/* ---------- API (stdin/stdout protocol for mms) ---------- */

/* Send a command and (optionally) read back a response line. */
static void api_send(const char *cmd)
{
    fprintf(stdout, "%s\n", cmd);
    fflush(stdout);
}

static int api_query_int(const char *cmd)
{
    api_send(cmd);
    char buf[64];
    if (!fgets(buf, sizeof(buf), stdin)) return 0;
    return atoi(buf);
}

static bool api_query_bool(const char *cmd)
{
    api_send(cmd);
    char buf[64];
    if (!fgets(buf, sizeof(buf), stdin)) return false;
    return (strncmp(buf, "true", 4) == 0);
}

static int  api_maze_width(void)  { return api_query_int("mazeWidth");  }
static int  api_maze_height(void) { return api_query_int("mazeHeight"); }
static bool api_wall_front(void)  { return api_query_bool("wallFront"); }
static bool api_wall_left(void)   { return api_query_bool("wallLeft");  }
static bool api_wall_right(void)  { return api_query_bool("wallRight"); }

static void api_move_forward(void)   { api_send("moveForward"); }
static void api_turn_left(void)      { api_send("turnLeft");    }
static void api_turn_right(void)     { api_send("turnRight");   }

/* mms has no "turnAround" command — do two left turns */
static void api_turn_around(void)
{
    api_send("turnLeft");
    api_send("turnLeft");
}

/* ---------- Maze (wall storage) ---------- */

/*
 * walls[x][y][d] == true  →  wall exists on side d of cell (x,y)
 * Walls are stored redundantly (both sides of a shared edge) for
 * simplicity, matching the Python implementation.
 */
static bool walls[MAX_WIDTH][MAX_HEIGHT][4];
static int  maze_w, maze_h;

static bool maze_contains(int x, int y)
{
    return (x >= 0 && x < maze_w && y >= 0 && y < maze_h);
}

static void maze_set_wall(int x, int y, Direction d)
{
    walls[x][y][d] = true;
}

static bool maze_get_wall(int x, int y, Direction d)
{
    return walls[x][y][d];
}

/* Center region: the 2x2 cells at the middle of the maze */
static bool maze_in_center(int x, int y)
{
    int cx = (maze_w - 1) / 2;
    int cy = (maze_h - 1) / 2;
    return ((x == cx || x == cx + 1) && (y == cy || y == cy + 1));
}

/* ---------- Mouse state ---------- */
static int       mouse_x, mouse_y;
static Direction mouse_dir;

/* ---------- BFS helpers ---------- */

typedef struct { int x, y; } Cell;

/* Flat index for a cell */
static int cell_idx(int x, int y) { return y * MAX_WIDTH + x; }

static void get_neighbor(int x, int y, Direction d, int *nx, int *ny)
{
    *nx = x; *ny = y;
    switch (d) {
        case NORTH: (*ny)++; break;
        case EAST:  (*nx)++; break;
        case SOUTH: (*ny)--; break;
        case WEST:  (*nx)--; break;
    }
}

/*
 * BFS from mouse position to find the next cell toward the center.
 * Returns the immediate neighbour to step into.
 */
static Cell get_next_cell(void)
{
    int initial_x = mouse_x, initial_y = mouse_y;

    /* ancestors[-1] means "no parent yet" */
    int ancestors_x[MAX_WIDTH * MAX_HEIGHT];
    int ancestors_y[MAX_WIDTH * MAX_HEIGHT];
    bool visited[MAX_WIDTH * MAX_HEIGHT];
    memset(visited,    0, sizeof(visited));
    memset(ancestors_x, -1, sizeof(ancestors_x));
    memset(ancestors_y, -1, sizeof(ancestors_y));

    Cell queue[QUEUE_CAP];
    int  head = 0, tail = 0;

    queue[tail].x = initial_x;
    queue[tail].y = initial_y;
    tail++;
    visited[cell_idx(initial_x, initial_y)] = true;

    Cell center = {-1, -1};

    while (head < tail && center.x == -1) {
        Cell cur = queue[head++];

        for (Direction d = NORTH; d <= WEST; d++) {
            int nx, ny;
            get_neighbor(cur.x, cur.y, d, &nx, &ny);

            if (!maze_contains(nx, ny))               continue;
            if (maze_get_wall(cur.x, cur.y, d))       continue;
            if (visited[cell_idx(nx, ny)])             continue;

            visited[cell_idx(nx, ny)] = true;
            ancestors_x[cell_idx(nx, ny)] = cur.x;
            ancestors_y[cell_idx(nx, ny)] = cur.y;
            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;

            if (maze_in_center(nx, ny)) {
                center.x = nx;
                center.y = ny;
                break;
            }
        }
    }

    /* Walk ancestors backward until the parent is the initial cell */
    Cell pos = center;
    while (ancestors_x[cell_idx(pos.x, pos.y)] != initial_x ||
           ancestors_y[cell_idx(pos.x, pos.y)] != initial_y) {
        int px = ancestors_x[cell_idx(pos.x, pos.y)];
        int py = ancestors_y[cell_idx(pos.x, pos.y)];
        pos.x = px;
        pos.y = py;
    }
    return pos;
}

/* ---------- Wall sensing ---------- */
static void update_walls(void)
{
    if (api_wall_front()) maze_set_wall(mouse_x, mouse_y, mouse_dir);
    if (api_wall_left())  maze_set_wall(mouse_x, mouse_y, turn_left(mouse_dir));
    if (api_wall_right()) maze_set_wall(mouse_x, mouse_y, turn_right(mouse_dir));
}

/* ---------- Movement ---------- */
static void move_one_cell(void)
{
    Cell next = get_next_cell();

    /* Determine the absolute direction to the next cell */
    Direction next_dir;
    if      (next.x < mouse_x) next_dir = WEST;
    else if (next.x > mouse_x) next_dir = EAST;
    else if (next.y < mouse_y) next_dir = SOUTH;
    else                        next_dir = NORTH;

    /* Rotate to face that direction */
    if (turn_left(mouse_dir) == next_dir) {
        api_turn_left();
        mouse_dir = turn_left(mouse_dir);
    } else if (turn_right(mouse_dir) == next_dir) {
        api_turn_right();
        mouse_dir = turn_right(mouse_dir);
    } else if (mouse_dir != next_dir) {
        api_turn_around();
        mouse_dir = turn_around(mouse_dir);
    }

    /* Step forward */
    api_move_forward();
    mouse_x = next.x;
    mouse_y = next.y;
}

/* ---------- main ---------- */
int main(void)
{
    /* Disable stdout buffering so mms sees output immediately */
    setbuf(stdout, NULL);

    maze_w = api_maze_width();
    maze_h = api_maze_height();
    memset(walls, 0, sizeof(walls));

    mouse_x   = 0;
    mouse_y   = 0;
    mouse_dir = NORTH;

    while (!maze_in_center(mouse_x, mouse_y)) {
        update_walls();
        move_one_cell();
    }

    return 0;
}