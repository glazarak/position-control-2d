import matplotlib.pyplot as plt
import numpy as np

steps = 2000
m = 1.0
g = 9.81
target_y = 10.0
target_x = 6.0
Kp = 2.0
Kd = 1.5

y = 0.0
x = 0.0
velocity_y = 0.0
velocity_x = 0.0
dt = 0.01

times = np.zeros(steps)
x_positions = np.zeros(steps)
y_positions = np.zeros(steps)

print("starting at", y, "m, aiming for", target_y, "m")
print("starting at", x, "m, aiming for", target_x, "m")

def pid(target, position, velocity, Kp, Kd):
    error = target - position
    force = Kp * error - Kd * velocity
    return force

for step in range(steps):
    thrust_y = m * g + pid(target_y, y, velocity_y, Kp, Kd)
    thrust_x = pid(target_x, x, velocity_x, Kp, Kd)
    net_force_y = thrust_y - m * g
    net_force_x = thrust_x
    acceleration_y = net_force_y / m
    acceleration_x = net_force_x / m
    velocity_y = velocity_y + acceleration_y * dt
    velocity_x = velocity_x + acceleration_x * dt
    y = y + velocity_y * dt
    x = x + velocity_x * dt
    y_positions[step] = y
    x_positions[step] = x
    times[step] = step * dt

    if step % 100 == 0:
        print(round(step * dt, 1), "s   y", round(y, 2), "m")
        print(round(step * dt, 1), "s   x", round(x, 2), "m")

plt.plot(times, y_positions, label="Y position")
plt.plot(times, x_positions, label="X position")
plt.axhline(10, linestyle="--", color="gray", label="target_y")
plt.axhline(6, linestyle="--", color="gray", label="target_x")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.title("2D-Control")
plt.legend()
plt.savefig("plot.png")
plt.show()