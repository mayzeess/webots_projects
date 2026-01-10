"""obezd controller."""
from controller import Robot, DistanceSensor, Motor

# Создание робота
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28  

# Инициализация датчиков расстояния
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Основной цикл
while robot.step(TIME_STEP) != -1:
    # Чтение датчиков
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Определение препятствий
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # Базовая скорость
    leftSpeed = MAX_SPEED
    rightSpeed = MAX_SPEED

    # Избегание препятствий
    if left_obstacle:
        # Поворот направо
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # Поворот налево
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

    # Установка скоростей
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)