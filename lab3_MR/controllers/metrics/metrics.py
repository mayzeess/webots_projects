from controller import Robot
import sys

TIME_STEP = 32

WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

class URController:
    def __init__(self):
        self.robot = Robot()

        # parameters
        self.counter = 0
        self.state = WAITING
        self.target_positions = [-1.88, -2.14, -2.38, -1.51]
        self.speed = 1.0

        # load speed from controller args (optional)
        args = sys.argv[1:]
        if args:
            try:
                self.speed = float(args[0])
            except:
                pass

        # hand motors
        self.hand_motors = [
            self.robot.getDevice("finger_1_joint_1"),
            self.robot.getDevice("finger_2_joint_1"),
            self.robot.getDevice("finger_middle_joint_1"),
        ]

        # arm motors
        self.ur_motors = [
            self.robot.getDevice("shoulder_lift_joint"),
            self.robot.getDevice("elbow_joint"),
            self.robot.getDevice("wrist_1_joint"),
            self.robot.getDevice("wrist_2_joint"),
        ]
        for m in self.ur_motors:
            m.setVelocity(self.speed)

        # sensors
        self.distance_sensor = self.robot.getDevice("distance sensor")
        self.distance_sensor.enable(TIME_STEP)

        self.position_sensor = self.robot.getDevice("wrist_1_joint_sensor")
        self.position_sensor.enable(TIME_STEP)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:

            if self.counter <= 0:

                if self.state == WAITING:
                    if self.distance_sensor.getValue() < 500:
                        print("Grasping can")
                        self.state = GRASPING
                        self.counter = 8
                        for m in self.hand_motors:
                            m.setPosition(0.85)

                elif self.state == GRASPING:
                    print("Rotating arm")
                    for i in range(4):
                        self.ur_motors[i].setPosition(self.target_positions[i])
                    self.state = ROTATING

                elif self.state == ROTATING:
                    if self.position_sensor.getValue() < -2.3:
                        print("Releasing can")
                        self.state = RELEASING
                        self.counter = 8
                        for m in self.hand_motors:
                            m.setPosition(m.getMinPosition())

                elif self.state == RELEASING:
                    print("Rotating arm back")
                    for m in self.ur_motors:
                        m.setPosition(0.0)
                    self.state = ROTATING_BACK

                elif self.state == ROTATING_BACK:
                    if self.position_sensor.getValue() > -0.1:
                        print("Waiting can")
                        self.state = WAITING

            self.counter -= 1


# main
controller = URController()
controller.run()
