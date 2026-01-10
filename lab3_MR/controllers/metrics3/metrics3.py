from controller import Robot
import csv

TIME_STEP = 32

WAITING = 0
GRASPING = 1
ROTATING = 2
RELEASING = 3
ROTATING_BACK = 4

MAX_WAIT_NO_CAN = 10.0

class URController:
    def __init__(self, speed=0.5, metrics_file="metrics2.csv"):
        self.robot = Robot()
        self.speed = speed
        self.counter = 0
        self.state = WAITING
        self.target_positions = [-1.88, -2.14, -2.38, -1.51]

        self.hand_motors = [
            self.robot.getDevice("finger_1_joint_1"),
            self.robot.getDevice("finger_2_joint_1"),
            self.robot.getDevice("finger_middle_joint_1"),
        ]
        self.ur_motors = [
            self.robot.getDevice("shoulder_lift_joint"),
            self.robot.getDevice("elbow_joint"),
            self.robot.getDevice("wrist_1_joint"),
            self.robot.getDevice("wrist_2_joint"),
        ]
        for m in self.ur_motors:
            m.setVelocity(self.speed)

        self.position_sensors = [
            self.robot.getDevice("shoulder_lift_joint_sensor"),
            self.robot.getDevice("elbow_joint_sensor"),
            self.robot.getDevice("wrist_1_joint_sensor"),
            self.robot.getDevice("wrist_2_joint_sensor"),
        ]
        for s in self.position_sensors:
            s.enable(TIME_STEP)

        self.distance_sensor = self.robot.getDevice("distance sensor")
        self.distance_sensor.enable(TIME_STEP)

        self.metrics_file = metrics_file
        self.cycles_completed = 0
        self.velocities_cycle = []
        self.acc_cycle = []
        self.wait_times = []
        self.errors_cycle = 0
        self.cycle_start_time = None

        self.cycle_times = []
        self.avg_velocities = []
        self.max_velocities = []
        self.avg_acc = []
        self.max_acc = []
        self.wait_times_list = []
        self.error_counts = []

        self.last_positions = [s.getValue() for s in self.position_sensors]
        self.last_velocities = [0]*len(self.position_sensors)
        self.wait_start_time = self.robot.getTime()

    def save_metrics(self):
        rows = min(10, len(self.cycle_times))
        with open(self.metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "cycle", "time", "avg_velocity", "max_velocity",
                "avg_acceleration", "max_acceleration", "waiting", "error"
            ])
            for i in range(rows):
                writer.writerow([
                    i+1,
                    self.cycle_times[i],
                    self.avg_velocities[i],
                    self.max_velocities[i],
                    self.avg_acc[i],
                    self.max_acc[i],
                    self.wait_times_list[i],
                    self.error_counts[i]
                ])
        print(f"Metrics saved to {self.metrics_file}")

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            current_time = self.robot.getTime()

            if self.state == WAITING:
                if current_time - self.wait_start_time > MAX_WAIT_NO_CAN:
                    self.save_metrics()
                    break

            if self.counter <= 0:
                if self.state == WAITING:
                    if self.distance_sensor.getValue() < 500:
                        self.state = GRASPING
                        self.counter = 8
                        for m in self.hand_motors:
                            m.setPosition(0.85)
                        self.cycle_start_time = current_time
                        self.velocities_cycle = []
                        self.acc_cycle = []
                        self.errors_cycle = 0
                        self.wait_times.append(current_time - self.wait_start_time)

                elif self.state == GRASPING:
                    for i in range(4):
                        self.ur_motors[i].setPosition(self.target_positions[i])
                    self.state = ROTATING

                elif self.state == ROTATING:
                    if self.position_sensors[2].getValue() < -2.3:
                        self.state = RELEASING
                        self.counter = 8
                        for m in self.hand_motors:
                            m.setPosition(m.getMinPosition())

                elif self.state == RELEASING:
                    for m in self.ur_motors:
                        m.setPosition(0.0)
                    self.state = ROTATING_BACK

                elif self.state == ROTATING_BACK:
                    if self.position_sensors[2].getValue() > -0.1:
                        self.state = WAITING
                        self.cycles_completed += 1
                        cycle_time = current_time - self.cycle_start_time
                        self.cycle_times.append(cycle_time)

                        if self.velocities_cycle:
                            avg_v = sum(self.velocities_cycle)/len(self.velocities_cycle)
                            max_v = max(self.velocities_cycle)
                        else:
                            avg_v = max_v = 0
                        if self.acc_cycle:
                            avg_a = sum(self.acc_cycle)/len(self.acc_cycle)
                            max_a = max(self.acc_cycle)
                        else:
                            avg_a = max_a = 0

                        self.avg_velocities.append(avg_v)
                        self.max_velocities.append(max_v)
                        self.avg_acc.append(avg_a)
                        self.max_acc.append(max_a)
                        self.error_counts.append(self.errors_cycle)
                        self.wait_times_list.append(self.wait_times.pop(0))
                        self.wait_start_time = current_time

            self.counter -= 1

            current_positions = [s.getValue() for s in self.position_sensors]
            dt = current_time - self.robot.getTime()
            if dt <= 0:
                dt = TIME_STEP / 1000.0
            velocities = [(p - lp)/dt for p, lp in zip(current_positions, self.last_positions)]
            acc = [(v - lv)/dt for v, lv in zip(velocities, self.last_velocities)]

            self.velocities_cycle.append(max(map(abs, velocities)))
            self.acc_cycle.append(max(map(abs, acc)))

            self.last_positions = current_positions
            self.last_velocities = velocities


controller = URController(speed=0.5, metrics_file="metrics2.csv")
controller.run()
