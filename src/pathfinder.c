#include <stdio.h>
#include <math.h>

#include "pathfinder.h"

path_status pathfinder(uint8_t *image, path_return_struct *path_return){

    int8_t  bot_row = 64;
    int8_t  bot_start = -1;
    int8_t  bot_ender = -1;
    int8_t  bot_center= -1;

    int8_t  mid_row = 0;
    int8_t  mid_start = -1;
    int8_t  mid_ender = -1;
    int8_t  mid_center= -1;

    int8_t  top_row = -5;
    int8_t  top_start = -1;
    int8_t  top_ender = -1;
    int8_t  top_center= -1;


    //find bottom
    while(bot_start == -1 && bot_ender == -1 && bot_row >= 5){
        bot_row = bot_row - 5;
        find_center_line(&bot_start, &bot_ender, &bot_center, image, bot_row);
    }

    //find top
    while(top_start == -1 && top_ender == -1 && top_row <= 54){
        top_row = top_row + 5;
        find_center_line(&top_start, &top_ender, &top_center, image, top_row);
    }

    // **** check if something has been found
    if(bot_start == -1){
        path_return->no_operations = 1;
        path_return->rotate1 = FOV_W/2;
        path_return->dist1 = 0;
        return PATH_NO_LINE;
    }

    // **** check if bottom reading is out of bound
    if(bot_row == 59 && (bot_start == 0 || bot_ender == 79)){

        int16_t angle = 0;
        float dist = 0;
        anlge_dist_from_point(&angle, &dist, 60, bot_center);

        path_return->no_operations = 1;
        path_return->rotate1 = angle;
        path_return->dist1 = dist;

        return PATH_SUCCESS;
    }


    // **** normal line with or without curve
    if(top_row == 0 && bot_row == 59){

        //find mid center
        mid_row = 30;
        find_center_line(&mid_start, &mid_ender, &mid_center, image, mid_row);

        //find angle and distance to top and mid
        int16_t top_angle = 0, mid_angle = 0;
        float top_dist = 0, mid_dist = 0;

        anlge_dist_from_point(&top_angle, &top_dist, top_row, top_center);
        anlge_dist_from_point(&mid_angle, &mid_dist, mid_row, mid_center);

        path_return->no_operations = 1;
        path_return->rotate1 = mid_angle;
        path_return->dist1 = mid_dist;

        //find angle between top and bottom
        int16_t mid_top_angle = 0;
        angle_two_points(&mid_top_angle, mid_angle, mid_dist, top_angle, top_dist);

        if((mid_top_angle < -10) || (mid_top_angle > 10)){
            path_return->no_operations = 2;
            path_return->rotate2 = mid_top_angle;
            path_return->dist2 = 0;
        }

        return PATH_SUCCESS;
    }

    // **** curve, cornor, or srub
    if(top_row > 0 && bot_row == 59){

        if(top_start == 0 || top_ender == 79){

            int8_t  temp1_start = bot_start, temp2_start = -1;
            int8_t  temp1_ender = bot_ender, temp2_ender = -1;
            int8_t  temp1_center= bot_center, temp2_center= -1;

            int dx_start = 0, dx_ender = 0;
            int row = 0;

            int8_t lines = (bot_row - top_row) / 5;

            for(int8_t i = 0; i < lines+1; i++){

                if(i != 0){
                    temp1_start = temp2_start;
                    temp1_ender = temp2_ender;
                    temp1_center = temp2_center;
                }

                if(i != lines){
                    row = bot_row - 5*(i+1);
                }
                else{
                    row = top_row;
                }

                find_center_line(&temp2_start, &temp2_ender, &temp2_center, image, row);

                dx_start = temp2_start - temp1_start;
                dx_ender = temp2_ender - temp1_ender;

                //corner left
                if(top_start == 0 && dx_start < -15){

                    //find angle and distance to top and mid
                    int16_t top_angle = 0, mid_angle = 0;
                    float top_dist = 0, mid_dist = 0;

                    mid_row = row + 5;
                    find_center_line(&mid_start, &mid_ender, &mid_center, image, mid_row);

                    anlge_dist_from_point(&top_angle, &top_dist, top_row, top_start);
                    anlge_dist_from_point(&mid_angle, &mid_dist, mid_row, mid_center);

                    int16_t mid_top_angle = 0;
                    angle_two_points(&mid_top_angle, mid_angle, mid_dist, top_angle, top_dist);

                    path_return->no_operations = 2;
                    path_return->rotate1 = mid_angle;
                    path_return->dist1 = mid_dist;
                    path_return->rotate2 = mid_top_angle;
                    path_return->dist2 = 0;

                    return PATH_SUCCESS;
                }

                //corner right
                if(top_ender == 79 && dx_ender > 15){

                    //find angle and distance to top and mid
                    int16_t top_angle = 0, mid_angle = 0;
                    float top_dist = 0, mid_dist = 0;

                    mid_row = row + 5;
                    find_center_line(&mid_start, &mid_ender, &mid_center, image, mid_row);

                    anlge_dist_from_point(&top_angle, &top_dist, top_row, top_ender);
                    anlge_dist_from_point(&mid_angle, &mid_dist, mid_row, mid_center);

                    int16_t mid_top_angle = 0;
                    angle_two_points(&mid_top_angle, mid_angle, mid_dist, top_angle, top_dist);

                    path_return->no_operations = 2;
                    path_return->rotate1 = mid_angle;
                    path_return->dist1 = mid_dist;
                    path_return->rotate2 = mid_top_angle;
                    path_return->dist2 = 0;

                    return PATH_SUCCESS;
                }

            }

        }

        //gentle curve
        int16_t top_angle = 0, mid_angle = 0;
        float top_dist = 0, mid_dist = 0;

        mid_row = top_row + (bot_row - top_row)/2;
        find_center_line(&mid_start, &mid_ender, &mid_center, image, mid_row);

        anlge_dist_from_point(&top_angle, &top_dist, top_row, top_center);
        anlge_dist_from_point(&mid_angle, &mid_dist, mid_row, mid_center);

        int16_t mid_top_angle = 0;
        angle_two_points(&mid_top_angle, mid_angle, mid_dist, top_angle, top_dist);

        path_return->no_operations = 2;
        path_return->rotate1 = mid_angle;
        path_return->dist1 = mid_dist;
        path_return->rotate2 = mid_top_angle;
        path_return->dist2 = 0;

        return PATH_SUCCESS;



    }
    //stub
    else{
        int16_t top_angle = 0, bot_angle = 0;
        float top_dist = 0, bot_dist = 0;

        anlge_dist_from_point(&top_angle, &top_dist, top_row, top_center);
        anlge_dist_from_point(&bot_angle, &bot_dist, bot_row, bot_center);

        int16_t top_bot_angle = 0;
        angle_two_points(&top_bot_angle, top_angle, top_dist, bot_angle, bot_dist);

        path_return->no_operations = 2;
        path_return->rotate1 = top_angle;
        path_return->dist1 = top_dist;
        /*
        path_return->rotate2 = top_bot_angle;
        path_return->dist2 = 0;
        */
        return PATH_SUCCESS;
    }


    //croosing segment
    int16_t top_angle = 0, mid_angle = 0;
    float top_dist = 0, mid_dist = 0;

    mid_row = top_row + (bot_row - top_row)/2;
    find_center_line(&mid_start, &mid_ender, &mid_center, image, mid_row);

    anlge_dist_from_point(&top_angle, &top_dist, top_row, top_center);
    anlge_dist_from_point(&mid_angle, &mid_dist, mid_row, mid_center);

    int16_t mid_top_angle = 0;
    angle_two_points(&mid_top_angle, mid_angle, mid_dist, top_angle, top_dist);

    path_return->no_operations = 1;
    path_return->rotate1 = mid_angle;
    path_return->dist1 = mid_dist;
/*
    if((mid_top_angle < -10) || (mid_top_angle > 10)){
        path_return->no_operations = 2;
        path_return->rotate2 = mid_top_angle;
        path_return->dist2 = 0;
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
                if(thickness > 5) l_flag = 2;
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
            c_end = 79;
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

    double phi_ph = PHI_CAM + (((IM_ROWS/2.0) - (double)pix_h) / IM_ROWS) * FOV_H;
    double phi_pw = (((IM_COLS/2.0) - (double)pix_w) / IM_COLS) * FOV_W;

    double l_pc = tan(deg2rad(phi_ph)) * Z_CAM;
    double l_pr = X_CAM + l_pc;

    double hyp_p = sqrt(Z_CAM*Z_CAM + l_pc*l_pc);
    double w_p   = tan(deg2rad(phi_pw)) * hyp_p;

    double angle_d = rad2deg(atan2(w_p,l_pr));
    double dist_d  = sqrt(l_pr*l_pr + w_p*w_p);

    *dist = (float)dist_d;
    *angle = (int16_t)angle_d;

    return PATH_SUCCESS;
}



path_status angle_two_points(int16_t *angle_out, int16_t angle1, float dist1, int16_t angle2, float dist2){

    double w1 = sin(deg2rad((double)angle1)) * dist1;
    double d1 = cos(deg2rad((double)angle1)) * dist1;

    double w2 = sin(deg2rad((double)angle2)) * dist2;
    double d2 = cos(deg2rad((double)angle2)) * dist2;

    double w3 = w2 - w1;
    double d3 = d2 - d1;

    double angle3 = rad2deg(atan2(w3, d3));

    *angle_out = angle3 - angle1;

    return PATH_SUCCESS;
}


double deg2rad(double deg){
    return (deg/180)*PI;
}



double rad2deg(double rad){
    return (rad/PI)*180;
}


