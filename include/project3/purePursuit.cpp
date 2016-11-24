#include <project3/purePursuit.h>
#include <cmath>
#include <stdio.h>

#define MAX_VEL_LIN 0.25
#define MAX_VEL_ANG M_PI / 180 * 300 / 10

#define MAX_VSQ_OVER_R 0.1

#define THRESHOLD_1 M_PI / 2
#define THRESHOLD_2 M_PI / 18

using namespace std;

purePursuit::purePursuit(){

}

control purePursuit::get_control(point x_robot, point x_goal){

    control ctrl;

    double delta_x = x_goal.x - x_robot.x;
    double delta_y = x_goal.y - x_robot.y;
    double delta_x_rel = delta_x * sin(x_robot.th) - delta_y * cos(x_robot.th);
    double delta_y_rel = delta_x * cos(x_robot.th) + delta_y * sin(x_robot.th);

    if (delta_x_rel == 0 && delta_y_rel < 0) {
      ctrl.v = MAX_VEL_LIN;
      ctrl.w = MAX_VEL_ANG;

      return ctrl;
    }

    double gamma = 2 * delta_x_rel / (delta_x * delta_x + delta_y * delta_y); // always positive
    double goalTh = M_PI / 2 - atan(delta_y/delta_x) * (delta_x_rel > 0 ? 1 : -1);
    double gamma_adj;

    if (goalTh > THRESHOLD_1) { // fast turn
    	gamma_adj = gamma * 10;
    }
    else if (goalTh > THRESHOLD_2) { // slow turn
    	gamma_adj = gamma * 5;
    }
    else {
    	gamma_adj = gamma * 1;
    }

    double v_rel = sqrt(MAX_VSQ_OVER_R / abs(gamma_adj));
    if (v_rel > MAX_VEL_LIN) {
        v_rel = MAX_VEL_LIN;
    }
    double w_rel = v_rel * gamma_adj;
    
    ctrl.v = v_rel;
    ctrl.w = w_rel;
    
    return (ctrl);
}