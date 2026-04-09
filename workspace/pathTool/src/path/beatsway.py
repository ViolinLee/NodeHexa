import math

from lib import get_rotate_x_matrix, get_rotate_y_matrix, get_rotate_z_matrix

g_steps = 20

roll_angle = 6
pitch_angle = 4
yaw_angle = 10
x_sway = 3
y_sway = 5
z_bounce = 3


def path_generator():
    pi = math.acos(-1)

    result = []
    for i in range(g_steps):
        t = 2 * pi * i / g_steps

        roll = roll_angle * math.sin(t)
        pitch = pitch_angle * math.sin(2 * t)
        yaw = yaw_angle * math.sin(t)

        m = get_rotate_z_matrix(yaw) * get_rotate_y_matrix(pitch) * get_rotate_x_matrix(roll)
        m[0, 3] = x_sway * math.sin(t)
        m[1, 3] = y_sway * math.sin(2 * t)
        m[2, 3] = z_bounce * (1 - math.cos(2 * t)) / 2
        result.append(m)

    return result, "matrix", 45, [0, int(g_steps / 2)]
