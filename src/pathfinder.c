
#include "pathfinder.h"
#include <stdio.h>
#include <math.h>

path_status pathfinder(uint8_t *image, path_return_struct *path_return){



    //find bottom
    int8_t  bot_row = 65;
    int8_t  bot_start = -1;
    int8_t  bot_ender = -1;
    int8_t  bot_center= -1;

    while(bot_start == -1 && bot_ender == -1 && bot_row >= 0){
        bot_row = bot_row - 5;
        find_center_line(&bot_start, &bot_ender, &bot_center, image, bot_row);
    }

    //find top
    int8_t  top_row = -4;
    int8_t  top_start = -1;
    int8_t  top_ender = -1;
    int8_t  top_center= -1;

    while(top_start == -1 && top_ender == -1 && top_row <= 60){
        top_row = top_row + 5;
        find_center_line(&top_start, &top_ender, &top_center, image, top_row);
    }

    //check if something has been found
    if(bot_start == -1){
        path_return->no_operations = 1;
        path_return->rotate1 = FOV_W/2;
        path_return->dist1 = 0;
        return PATH_NO_LINE;
    }

    int16_t angle_1;
    float dist_1;
    anlge_dist_from_point(&angle_1, &dist_1, 60, 20);

    /*
    //check if bottom reading is out of bound
    if(bot_row == 60 && (bot_start == 1 || bot_ender == 80)){

        int16_t angle_1;
        float dist_1;
        //path_status anlge_dist_from_point(int16_t *angle, float *dist, uint8_t pix_h, uint8_t pix_w)

        path_return->no_operations = 1;
        path_return->rotate1 = angle;
        path_return->dist1 = dist;

        return PATH_SUCCESS;
    }
    */

    return PATH_SUCCESS;
}


path_status find_center_line(int8_t *start, int8_t *ender, int8_t *center, uint8_t *image, uint8_t row){

    uint8_t l_flag = 0;
    int8_t  c_end = -1;
    int8_t  c_start = -1;
    uint8_t thickness = 0;

    for(int8_t i = 0; i < IM_COLS; i++){

        uint8_t data = image[ (IM_COLS*row) + i ];

        //find start edge
        if(l_flag == 0){

            if(data < IM_THRES){
                c_start = i;
                thickness = 1;
                l_flag = 1;
            }
        }
        //check length
        else if(l_flag == 1){

            if( data < IM_THRES ){
                thickness = thickness + 1;
                if(thickness > 5)l_flag = 2;
            }
            else{
                thickness = 0;
                c_start = -1;
                l_flag = 0;
            }

        }
        //find end edge
        else if(l_flag == 2){
            if(data > IM_THRES){
                c_end = i;
                l_flag = 3;
            }
        }
    }

    //
    if((c_start > 0) && (c_end < 0)){
        if(l_flag == 2){
            c_end = 80;
        }
        else{
            c_start = -1;
        }
    }

    *center = c_start + ((c_end - c_start) / 2);
    *start = c_start;
    *ender = c_end;

    return PATH_SUCCESS;
}


path_status anlge_dist_from_point(int16_t *angle, float *dist, uint8_t pix_h, uint8_t pix_w){

    //double phi_ph = PHI_CAM + (((IM_ROWS/2.0) - (double)pix_h) / IM_ROWS) * FOV_H;
    //double phi_pw = (((IM_COLS/2.0) - (double)pix_w) / IM_COLS) * FOV_W;


    double x = 50;
    double y = 0;
    y = tan(x);


    //double l_pc = tan(deg2rad(phi_ph)) * Z_CAM;
    /*
    double l_pr = X_CAM + l_pc;


    double hyp_p = sqrt(Z_CAM*Z_CAM + l_pc*l_pc);
    double w_p   = tan(deg2rad(phi_pw)) * hyp_p;

    double angle_d = rad2deg(atan2(w_p,l_pr));
    double dist_d  = sqrt(l_pr*l_pr + w_p*w_p);
*/
    //*dist = (float)dist_d;
    //*angle = (int16_t)angle_d;

    return PATH_SUCCESS;
}


double deg2rad(double deg){
    return (deg/180)*PI;
}

double rad2deg(double rad){
    return (rad/PI)*180;
}

