#ifndef __PATHFINDER_H
#define __PATHFINDER_H

#include <stdio.h>

#define X_CAM       0.07
#define Z_CAM       0.07
#define PHI_CAM     45
#define FOV_W       44.8
#define FOV_H       33.6
#define IM_ROWS     60
#define IM_COLS     80


typedef struct{
    uint8_t no_operations;
    float dist1;
    int16_t rotate1;
    float dist2;
    int16_t rotate2;
} path_return_struct;

typedef enum{
    PATH_SUCCESS = 0,
    PATH_FAIL = -1,
    PATH_NO_LINE = -2
} path_status;


path_status pathfinder(uint8_t *image, path_return_struct *path_return);




#endif /* __PATHFINDER_H */
