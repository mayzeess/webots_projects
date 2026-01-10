from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin

TARGET_HEIGHT_CM = 9.1
TARGET_DISTANCE_CM = 7.2
FORWARD_SPEED = 0.5
ALTITUDE_TOLERANCE = 0.01

STATE_TAKEOFF = 0
STATE_FORWARD = 1
STATE_HOVER = 2

STATE_NAMES = {
    STATE_TAKEOFF: "TAKEOFF",
    STATE_FORWARD: "FORWARD",
    STATE_HOVER:  "HOVER",
}


class VelocityHeightPID:
    def __init__(self): 
        self.kp_att_y = 1.0
        self.kd_att_y = 0.5
        self.kp_att_rp = 0.5
        self.kd_att_rp = 0.1
        self.kp_vel_xy = 2.0
        self.kd_vel_xy = 0.5
        self.kp_z = 10.0
        self.ki_z = 5.0
        self.kd_z = 5.0
        self.prev_vx_error = 0.0
        self.prev_vy_error = 0.0
        self.prev_alt_error = 0.0
        self.prev_pitch_error = 0.0
        self.prev_roll_error = 0.0
        self.altitude_integrator = 0.0

    @staticmethod
    def _clip(value, low, high):
        if value < low:
            return low
        if value > high:
            return high
        return value

    def update(self, dt,
               desired_vx, desired_vy, desired_yaw_rate, desired_altitude,
               actual_roll, actual_pitch, actual_yaw_rate,
               actual_altitude, actual_vx, actual_vy):

        if dt <= 0.0:
            dt = 1e-3

        vx_error = desired_vx - actual_vx
        vy_error = desired_vy - actual_vy

        vx_deriv = (vx_error - self.prev_vx_error) / dt
        vy_deriv = (vy_error - self.prev_vy_error) / dt

        desired_pitch = (
            self.kp_vel_xy * self._clip(vx_error, -1.0, 1.0)
            + self.kd_vel_xy * vx_deriv
        )
        desired_roll = (
            -self.kp_vel_xy * self._clip(vy_error, -1.0, 1.0)
            - self.kd_vel_xy * vy_deriv
        )

        self.prev_vx_error = vx_error
        self.prev_vy_error = vy_error

        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.prev_alt_error) / dt
        self.altitude_integrator += alt_error * dt

        integral_term = self._clip(self.altitude_integrator, -2.0, 2.0)
        alt_command = (
            self.kp_z * alt_error
            + self.kd_z * alt_deriv
            + self.ki_z * integral_term
            + 48.0
        )
        self.prev_alt_error = alt_error
        pitch_error = desired_pitch - actual_pitch
        roll_error = desired_roll - actual_roll

        pitch_deriv = (pitch_error - self.prev_pitch_error) / dt
        roll_deriv = (roll_error - self.prev_roll_error) / dt

        yaw_rate_error = desired_yaw_rate - actual_yaw_rate

        roll_command = (
            self.kp_att_rp * self._clip(roll_error, -1.0, 1.0)
            + self.kd_att_rp * roll_deriv
        )
        pitch_command = (
            -self.kp_att_rp * self._clip(pitch_error, -1.0, 1.0)
            - self.kd_att_rp * pitch_deriv
        )
        yaw_command = self.kp_att_y * self._clip(yaw_rate_error, -1.0, 1.0)

        self.prev_pitch_error = pitch_error
        self.prev_roll_error = roll_error

        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command
        m1 = self._clip(m1, 0.0, 600.0)
        m2 = self._clip(m2, 0.0, 600.0)
        m3 = self._clip(m3, 0.0, 600.0)
        m4 = self._clip(m4, 0.0, 600.0)

        return m1, m2, m3, m4


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    imu = robot.getDevice("inertial_unit")
    if imu is None:
        print("ERROR: inertial_unit device not found")
        return
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    if gps is None:
        print("ERROR: gps device not found")
        return
    gps.enable(timestep)

    gyro = robot.getDevice("gyro")
    if gyro is None:
        print("ERROR: gyro device not found")
        return
    gyro.enable(timestep)

    m1 = robot.getDevice("m1_motor")
    m2 = robot.getDevice("m2_motor")
    m3 = robot.getDevice("m3_motor")
    m4 = robot.getDevice("m4_motor")

    motors = [m1, m2, m3, m4]
    for i, m in enumerate(motors):
        if m is None:
            print(f"ERROR: motor m{i+1} not found")
            return
        m.setPosition(float('inf'))
        m.setVelocity(0.0)

    keyboard = Keyboard()
    keyboard.enable(timestep)

    robot.step(timestep)

    gps_values = gps.getValues()
    start_x = gps_values[0]
    start_y = gps_values[1]
    start_altitude = gps_values[2]

    target_height = start_altitude + TARGET_HEIGHT_CM / 100.0 
    target_distance = TARGET_DISTANCE_CM / 100.0

    pid = VelocityHeightPID()

    past_time = robot.getTime()
    past_x = start_x
    past_y = start_y

    state = STATE_TAKEOFF
    state_time = 0.0
    forward_start_x = None
    forward_distance = 0.0

    while robot.step(timestep) != -1:
        now = robot.getTime()
        dt = now - past_time
        if dt <= 0.0:
            dt = timestep / 1000.0
        past_time = now

        rpy = imu.getRollPitchYaw()
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        gyro_values = gyro.getValues()
        yaw_rate = gyro_values[2]

        gps_values = gps.getValues()
        x_global = gps_values[0]
        y_global = gps_values[1]
        altitude = gps_values[2]

        vx_global = (x_global - past_x) / dt
        vy_global = (y_global - past_y) / dt

        past_x = x_global
        past_y = y_global

        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        vx_body = vx_global * cosyaw + vy_global * sinyaw
        vy_body = -vx_global * sinyaw + vy_global * cosyaw

        state_time += dt

        desired_vx = 0.0
        desired_vy = 0.0
        desired_yaw_rate = 0.0
        desired_altitude = target_height

        key = keyboard.getKey()
        while key > 0:
            if key == ord('R'):
                state = STATE_TAKEOFF
                state_time = 0.0
                forward_start_x = None
                start_altitude = altitude
                target_height = start_altitude + TARGET_HEIGHT_CM / 100.0
                pid = VelocityHeightPID()

            key = keyboard.getKey()

        if state == STATE_TAKEOFF:
            if abs(altitude - target_height) < ALTITUDE_TOLERANCE and state_time > 0.5:
                state = STATE_FORWARD
                state_time = 0.0
                forward_start_x = x_global
                forward_distance = 0.0

        elif state == STATE_FORWARD:
            desired_vx = FORWARD_SPEED
            if forward_start_x is None:
                forward_start_x = x_global
                forward_distance = 0.0
            else:
                forward_distance = abs(x_global - forward_start_x)
            
            if forward_distance >= target_distance:
                state = STATE_HOVER
                state_time = 0.0
                desired_vx = 0.0

        elif state == STATE_HOVER:
            desired_vx = 0.0

        m1_cmd, m2_cmd, m3_cmd, m4_cmd = pid.update(
            dt,
            desired_vx, desired_vy, desired_yaw_rate, desired_altitude,
            roll, pitch, yaw_rate, altitude,
            vx_body, vy_body
        )

        m1.setVelocity(-m1_cmd)
        m2.setVelocity(m2_cmd)
        m3.setVelocity(-m3_cmd)
        m4.setVelocity(m4_cmd)

        alt_error = target_height - altitude
        debug_state = STATE_NAMES.get(state, "UNKNOWN")


if __name__ == "__main__":
    main()