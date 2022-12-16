#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32
double t = 0.0;         // elapsed simulation time
double v = 6.28;        // max velocity

void drive(WbDeviceTag rm, WbDeviceTag lm) {
  wb_motor_set_velocity(rm, v);
  wb_motor_set_velocity(lm, v);
}


void stop(WbDeviceTag rm, WbDeviceTag lm) {
  wb_motor_set_velocity(rm, 0);
  wb_motor_set_velocity(lm, 0);
} 

void turnLeft(WbDeviceTag rm, WbDeviceTag lm) {
  double time = 0.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    if(time < 0.07) {
      wb_motor_set_velocity(rm, v);
      wb_motor_set_velocity(lm, -v);
    } else {
      wb_motor_set_velocity(rm, 0);
      wb_motor_set_velocity(lm, 0);
      t+= time;
      return;
    }
    time += (double)TIME_STEP / 1000.0;
  }
}

void turnRight(WbDeviceTag rm, WbDeviceTag lm) {
  double time = 0.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    if(time < 0.07) {
      wb_motor_set_velocity(rm, -v);
      wb_motor_set_velocity(lm, v);
    } else {
      wb_motor_set_velocity(rm, 0);
      wb_motor_set_velocity(lm, 0);
      t += time;
      return;
    }
    time += (double)TIME_STEP / 1000.0;
  }
}





int followTheLine(WbDeviceTag rm, WbDeviceTag lm, int gl, int gc, int gr, double time, int counter) {
  if(gl > 800 && gc < 350 && gr > 800 ) {
    drive(rm, lm);
    return -1;
  }
  
  if(gl < 350 && gc > 800 && gr > 800) {
    turnLeft(rm, lm);
    return -1;
  }
  
  if(gl > 800 && gc > 800 && gr < 350) {
    turnRight(rm, lm);
    return -1;
  }
  
  if(gl < 350 && gc < 350 && gr < 350) {
    if(counter > 15 && time > 3.0) {
      stop(rm, lm);
      return 2;
    }
    drive(rm, lm);
    return 1;
  }
  drive(rm, lm);
  return -1;
}

int main() {
  wb_robot_init();
  // Initialization of the two wheel motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  
  // Initialization of the ground sensors
  WbDeviceTag gs_left = wb_robot_get_device("gs0");
  WbDeviceTag gs_center = wb_robot_get_device("gs1");
  WbDeviceTag gs_right = wb_robot_get_device("gs2");
  wb_distance_sensor_enable(gs_left, TIME_STEP);
  wb_distance_sensor_enable(gs_center, TIME_STEP);
  wb_distance_sensor_enable(gs_right, TIME_STEP);

  
  
  int c = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    int gl = wb_distance_sensor_get_value(gs_left);
    int gc = wb_distance_sensor_get_value(gs_center);
    int gr = wb_distance_sensor_get_value(gs_right);
    
    int s = followTheLine(right_motor, left_motor, gl, gc, gr, t, c);
    if(s==2){break;}
    c += s; 
    if(c < 0) {c = 0;}
    
    t += (double)TIME_STEP / 1000.0;
    /*printf("left/center/right = %d/%d/%d\n", \
            gl, gc, gr);
    printf("%i\n", c);*/
    printf("%.2f\n", t);
  }

  wb_robot_cleanup();
  return 0;
  
}
