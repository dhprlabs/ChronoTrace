import matplotlib.pyplot as plt


trajectory_x = []
trajectory_y = []


def log_trajectory(point):
    trajectory_x.append(point[0])
    trajectory_y.append(point[1])


def save_trajectory_plot(filename="trajectory.png"):
    plt.figure(figsize=(8, 6))

    plt.plot(
        trajectory_x,
        trajectory_y,
        linewidth=2,
        label="Trajectory"
    )

    plt.scatter(
        trajectory_x[0],
        trajectory_y[0],
        s=80,
        label="Start"
    )

    plt.scatter(
        trajectory_x[-1],
        trajectory_y[-1],
        s=80,
        label="End"
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("End Effector Trajectory")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    print(f"Trajectory saved as: {filename}")
    plt.show()