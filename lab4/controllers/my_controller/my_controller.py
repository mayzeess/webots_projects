# arm_controller.py
from controller import Robot, Motor, DistanceSensor, PositionSensor
import sys

TIME_STEP = 32

WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4


def main():
    robot = Robot()

    counter = 0
    state = WAITING

    target_positions = [-1.88, -2.14, -2.38, -1.51]
    speed = 1.0

    # speed can be passed as controller argument (same as in C)
    if len(sys.argv) == 2:
        try:
            speed = float(sys.argv[1])
        except ValueError:
            pass

    # Hand (gripper) motors
    hand_motor_names = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
    hand_motors = [robot.getDevice(name) for name in hand_motor_names]

    # UR arm motors
    ur_motor_names = ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
    ur_motors = [robot.getDevice(name) for name in ur_motor_names]

    # Set velocities for arm motors (same as C)
    for m in ur_motors:
        m.setVelocity(speed)

    # Distance sensor
    distance_sensor = robot.getDevice("distance sensor")
    distance_sensor.enable(TIME_STEP)

    # Position sensor (wrist_1_joint_sensor)
    position_sensor = robot.getDevice("wrist_1_joint_sensor")
    position_sensor.enable(TIME_STEP)

    while robot.step(TIME_STEP) != -1:
        if counter <= 0:
            if state == WAITING:
                if distance_sensor.getValue() < 500:
                    state = GRASPING
                    counter = 8
                    print("Grasping can")
                    for m in hand_motors:
                        m.setPosition(0.85)

            elif state == GRASPING:
                for i in range(4):
                    ur_motors[i].setPosition(target_positions[i])
                print("Rotating arm")
                state = ROTATING

            elif state == ROTATING:
                if position_sensor.getValue() < -2.3:
                    counter = 8
                    print("Releasing can")
                    state = RELEASING
                    for m in hand_motors:
                        m.setPosition(m.getMinPosition())

            elif state == RELEASING:
                for m in ur_motors:
                    m.setPosition(0.0)
                print("Rotating arm back")
                state = ROTATING_BACK

            elif state == ROTATING_BACK:
                if position_sensor.getValue() > -0.1:
                    state = WAITING
                    print("Waiting can")

        counter -= 1


if __name__ == "__main__":
    main()
