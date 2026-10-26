import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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

print("starting at", y, "m, aiming for", target_y, "m")
print("starting at", x, "m, aiming for", target_x, "m")

def pid(target, position, velocity, Kp, Kd):
    error = target - position
    force = Kp * error - Kd * velocity
    return force

def simulate(x, y, velocity_x, velocity_y, target_x, target_y, steps):
    data = np.zeros((steps, 8))
    for step in range(steps):
        thrust_y = m * g + pid(target_y, y, velocity_y, Kp, Kd)
        thrust_x = pid(target_x, x, velocity_x, Kp, Kd)
        data[step] = [x, y, velocity_x, velocity_y, target_x, target_y, thrust_x, thrust_y]
        net_force_y = thrust_y - m * g
        net_force_x = thrust_x
        acceleration_y = net_force_y / m
        acceleration_x = net_force_x / m
        velocity_y = velocity_y + acceleration_y * dt
        velocity_x = velocity_x + acceleration_x * dt
        y = y + velocity_y * dt
        x = x + velocity_x * dt

    return data


data = simulate(x, y, velocity_x, velocity_y, target_x, target_y, steps)

times = np.arange(steps) * dt
y_positions = data[:, 1]
x_positions = data[:, 0]

plt.plot(times, y_positions, label="Y position")
plt.plot(times, x_positions, label="X position")
plt.axhline(10, linestyle="--", color="gray", label="target_y")
plt.axhline(6, linestyle="--", color="gray", label="target_x")
plt.xlabel("time (s)")
plt.ylabel("distance (m)")
plt.title("2D-Control")
plt.legend()
plt.savefig("plot.png")

plt.figure()
plt.plot(x_positions, y_positions, label="Trajectory")
plt.plot(target_x, target_y, "o", color="red", label="target")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("2D Flight Path")
plt.axis("equal")
plt.legend()
plt.savefig("path.png")



np.random.seed(0)

flights = []
flight_steps = 500
for flight in range (20):
    start_x = np.random.uniform(-10, 10)
    start_y = np.random.uniform(0, 15)
    start_velocity_x = np.random.uniform(-3, 3)
    start_velocity_y = np.random.uniform(-3, 3)
    goal_x = np.random.uniform(-10, 10)
    goal_y = np.random.uniform(0, 15)
    flights.append(simulate(start_x, start_y, start_velocity_x, start_velocity_y, goal_x, goal_y, flight_steps))

all_data = np.vstack(flights)

df = pd.DataFrame(all_data, columns=["x", "y", "velocity_x", "velocity_y", "target_x", "target_y", "thrust_x", "thrust_y"])
df.to_csv("flight_data.csv", index=False)
print(df.shape)
print(df.head())

plt.figure()
for flight in flights:
    plt.plot(flight[:,0], flight[:,1], linewidth=0.8)
    plt.plot(flight[0,4], flight[0,5], "o", color="red", markersize=4)

plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("20 flights from random states")
plt.axis("equal")
plt.savefig("dataset.png")

plt.show()