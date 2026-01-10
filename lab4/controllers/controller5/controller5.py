# line_p_khepera3_fixed.py
from controller import Robot

TIME_STEP = 32

BASE_SPEED = 4.0          # чуть медленнее для стабильности
KP = 8.0                  # можно потом подбирать
MAX_SPEED = 12.0
CALIBRATION_TIME_S = 2.0  # чуть дольше
LINE_IS_BRIGHT = True     # если не едет - попробуй False

# если сенсоры "не отличаются", считаем что линия потеряна
LOST_EPS = 0.03
# скорость поиска линии
SEARCH_TURN = 3.0

robot = Robot()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

CANDIDATE_NAMES = [
    "gs0", "gs1", "gs2",
    "gs3", "gs4", "gs5",
    "ground sensor 0", "ground sensor 1", "ground sensor 2",
    "ground_sensor_0", "ground_sensor_1", "ground_sensor_2",
    "line sensor 0", "line sensor 1", "line sensor 2",
    "line_sensor_0", "line_sensor_1", "line_sensor_2",
    "ls0", "ls1", "ls2"
]

sensors = []
sensor_names = []
for name in CANDIDATE_NAMES:
    try:
        d = robot.getDevice(name)
        d.enable(TIME_STEP)
        sensors.append(d)
        sensor_names.append(name)
    except Exception:
        pass

if len(sensors) < 2:
    print("ERROR: Could not find at least 2 ground/light sensors on Khepera3.")
    print("Tried names:", CANDIDATE_NAMES)
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    while robot.step(TIME_STEP) != -1:
        pass
    raise SystemExit

# берём 3 (левый/центр/правый) если есть
if len(sensors) >= 3:
    sensors = sensors[:3]
    sensor_names = sensor_names[:3]

print("Using sensors:", sensor_names)

mins = [float("inf")] * len(sensors)
maxs = [float("-inf")] * len(sensors)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def normalize(v, vmin, vmax):
    span = vmax - vmin
    if span < 1e-6:
        return 0.5
    x = (v - vmin) / span
    return clamp(x, 0.0, 1.0)

# --- CALIBRATION: стоим на месте и "качаемся", чтобы увидеть разные цвета ---
calib_steps = int((CALIBRATION_TIME_S * 1000) / TIME_STEP)
for k in range(calib_steps):
    # покачивание: левое колесо вперёд, правое назад и наоборот
    if k < calib_steps // 2:
        left_motor.setVelocity(1.5)
        right_motor.setVelocity(-1.5)
    else:
        left_motor.setVelocity(-1.5)
        right_motor.setVelocity(1.5)

    if robot.step(TIME_STEP) == -1:
        break

    raw = [s.getValue() for s in sensors]
    for i, v in enumerate(raw):
        if v < mins[i]:
            mins[i] = v
        if v > maxs[i]:
            maxs[i] = v

print("Calibration done.")
print("mins:", mins)
print("maxs:", maxs)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# запомним последнее направление ошибки для поиска линии
last_e = 0.0
debug_counter = 0

while robot.step(TIME_STEP) != -1:
    raw = [s.getValue() for s in sensors]
    norm = [normalize(raw[i], mins[i], maxs[i]) for i in range(len(sensors))]

    if not LINE_IS_BRIGHT:
        norm = [1.0 - x for x in norm]

    # DEBUG: раз в ~0.5 секунды печатаем значения
    debug_counter += 1
    if debug_counter % 16 == 0:
        print("raw:", ["%.3f" % v for v in raw], " norm:", ["%.2f" % v for v in norm])

    if len(norm) == 3:
        left, center, right = norm
        ssum = left + center + right + 1e-9
        e = (-1.0 * left + 0.0 * center + 1.0 * right) / ssum
        strength = max(left, center, right) - min(left, center, right)
    else:
        left, right = norm[0], norm[1]
        e = (right - left)
        strength = abs(e)

    # если датчики почти одинаковые -> линия потеряна -> ищем
    if strength < LOST_EPS:
        turn = SEARCH_TURN if last_e >= 0 else -SEARCH_TURN
        vL = clamp(BASE_SPEED - turn, -MAX_SPEED, MAX_SPEED)
        vR = clamp(BASE_SPEED + turn, -MAX_SPEED, MAX_SPEED)
        left_motor.setVelocity(vL)
        right_motor.setVelocity(vR)
        continue

    last_e = e

    u = KP * e
    vL = clamp(BASE_SPEED - u, -MAX_SPEED, MAX_SPEED)
    vR = clamp(BASE_SPEED + u, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(vL)
    right_motor.setVelocity(vR)
