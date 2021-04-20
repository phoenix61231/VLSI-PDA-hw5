#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

struct CPOINTs{
    int cor_x, cor_y;

    struct CPOINTs *prev;
    struct CPOINTs *next;
};
typedef struct CPOINTs CPOINT;

struct NETs{
    int net_id;
    int pin_x[2];
    int pin_y[2];
    int manh_dis;

    int congest_cnt;
    bool congested;

    CPOINT *route;
};
typedef struct NETs NET;

struct BINs{
    int cor_x, cor_y;
    int T_cap, B_cap, L_cap, R_cap;
    int T_occ, B_occ, L_occ, R_occ;
    int T_hp, B_hp, L_hp, R_hp; 

    bool routed;
    int maze_cost;
    bool maze_flag;    

    struct BINs *T_next;
    struct BINs *B_next;
    struct BINs *L_next;
    struct BINs *R_next;
};
typedef struct BINs BIN;

FILE *R_file(char *file, char *mode);