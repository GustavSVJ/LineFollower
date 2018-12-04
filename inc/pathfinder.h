#ifndef __PATHFINDER_H
#define __PATHFINDER_H


#define PI          3.14159265359
#define X_CAM       0.075
#define Z_CAM       0.095
#define PHI_CAM     42.0
#define FOV_W       44.8
#define FOV_H       33.6
#define IM_ROWS     60
#define IM_COLS     80
#define IM_THRES    87

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
path_status find_center_line(int8_t *start, int8_t *ender, int8_t *center, uint8_t *image, uint8_t row);


path_status anlge_dist_from_point(int16_t *angle, float *dist, uint8_t pix_h, uint8_t pix_w);


path_status angle_two_points(int16_t *angle_out, int16_t angle1, float dist1, int16_t angle2, float dist2);

double deg2rad(double deg);
double rad2deg(double rad);

#endif /* __PATHFINDER_H */
