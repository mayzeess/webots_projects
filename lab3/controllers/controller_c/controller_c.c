#include <math.h>
#include <stdio.h>

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Подключение PID контроллера (у тебя уже есть pid_controller.h)
#include "pid_controller.h"

#define TIME_STEP 32

#define TARGET_HEIGHT 0.091   // 9.1 см
#define TARGET_DISTANCE 0.072 // 7.2 см

int main() {
  wb_robot_init();

  // Инициализация моторов
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");

  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_position(m4_motor, INFINITY);

  wb_motor_set_velocity(m1_motor, 0);
  wb_motor_set_velocity(m2_motor, 0);
  wb_motor_set_velocity(m3_motor, 0);
  wb_motor_set_velocity(m4_motor, 0);

  // Инициализация сенсоров
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag gyro = wb_robot_get_device("gyro");

  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);

  // Инициализация PID
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;

  init_pid_attitude_fixed_height_controller();

  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  motor_power_t motor_power;

  double past_x = 0;
  double phase = 0; // 0 - подъем, 1 - движение, 2 - удержание
  double start_x = 0;
  double past_time = wb_robot_get_time();

  while (wb_robot_step(TIME_STEP) != -1) {
    double current_time = wb_robot_get_time();
    double dt = current_time - past_time;

    // Получаем показания сенсоров
    double *imu_rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double *gyro_vals = wb_gyro_get_values(gyro);
    double *gps_vals = wb_gps_get_values(gps);

    actual_state.roll = imu_rpy[0];
    actual_state.pitch = imu_rpy[1];
    actual_state.yaw_rate = gyro_vals[2];
    actual_state.altitude = gps_vals[2];

    double x_global = gps_vals[0];
    double y_global = gps_vals[1];
    double vx_global = (x_global - past_x) / dt;
    past_x = x_global;

    // Вычисление скорости в координатах тела
    double yaw = imu_rpy[2];
    double cosyaw = cos(yaw);
    double sinyaw = sin(yaw);
    actual_state.vx = vx_global * cosyaw; // Движение вперед
    actual_state.vy = vx_global * sinyaw;

    // Определяем фазу выполнения
    if (phase == 0) {
      // Подъем до высоты TARGET_HEIGHT
      desired_state.altitude = TARGET_HEIGHT;
      desired_state.vx = 0;
      desired_state.vy = 0;
      if (actual_state.altitude >= TARGET_HEIGHT - 0.001) {
        phase = 1; // начинаем движение вперед
        start_x = x_global;
      }
    } else if (phase == 1) {
      // Движение вперед
      desired_state.altitude = TARGET_HEIGHT;
      desired_state.vx = 0.05; // небольшая скорость вперед
      desired_state.vy = 0;
      if ((x_global - start_x) >= TARGET_DISTANCE) {
        phase = 2; // достигли цели
      }
    } else if (phase == 2) {
      // Удержание позиции
      desired_state.altitude = TARGET_HEIGHT;
      desired_state.vx = 0;
      desired_state.vy = 0;
    }

    // PID контроллер
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // Установка скорости моторов
    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    past_time = current_time;
  }

  wb_robot_cleanup();
  return 0;
}
