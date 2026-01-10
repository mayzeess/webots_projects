from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

Kp = 0.003
base_speed = 3.0
max_speed = 6.28

sensor_names = [
    "ground left infrared sensor",
    "ground front left infrared sensor",
    "ground front right infrared sensor",
    "ground right infrared sensor"
]

# Подключение сенсоров
sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    sensors.append(sensor)

# Подключение моторов
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Переход в режим задания скорости (а не позиции)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Основной цикл
while robot.step(timestep) != -1:
    # Чтение значений с сенсоров (отражённый свет)
    left_value         = sensors[0].getValue()
    center_left_value  = sensors[1].getValue()
    center_right_value = sensors[2].getValue()
    right_value        = sensors[3].getValue()

    left_side  = left_value + center_left_value
    right_side = right_value + center_right_value

    error = right_side - left_side

    correction = Kp * error

    left_speed  = base_speed + correction
    right_speed = base_speed - correction

    # Ограничение скоростей
    if left_speed > max_speed:
        left_speed = max_speed
    elif left_speed < -max_speed:
        left_speed = -max_speed

    if right_speed > max_speed:
        right_speed = max_speed
    elif right_speed < -max_speed:
        right_speed = -max_speed

    # Установка скоростей моторов
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
