"""2-link planar robot arm: forward kinematics, inverse kinematics, animated path."""
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

L1, L2 = 1.0, 0.7  # link lengths (m)


def forward(t1, t2):
    """Joint angles (rad) -> elbow and end-effector positions."""
    x1, y1 = L1 * np.cos(t1), L1 * np.sin(t1)
    x2, y2 = x1 + L2 * np.cos(t1 + t2), y1 + L2 * np.sin(t1 + t2)
    return (x1, y1), (x2, y2)


def inverse(x, y, elbow=1):
    """End-effector position -> joint angles (rad). elbow=+1 or -1 picks the solution."""
    c2 = (x * x + y * y - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1:
        raise ValueError(f"Point ({x:.2f}, {y:.2f}) is outside the reachable workspace")
    s2 = elbow * np.sqrt(1 - c2**2)
    t2 = np.arctan2(s2, c2)
    t1 = np.arctan2(y, x) - np.arctan2(L2 * s2, L1 + L2 * c2)
    return t1, t2


def circle_path(n=150, cx=1.0, cy=0.3, r=0.4):
    a = np.linspace(0, 2 * np.pi, n)
    return cx + r * np.cos(a), cy + r * np.sin(a)


def line_path(n=120, p0=(1.4, 0.2), p1=(0.3, 1.1)):
    return np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n)


def animate(xs, ys, save=None):
    angles = np.array([inverse(x, y) for x, y in zip(xs, ys)])
    deg = np.degrees(angles)
    reach = L1 + L2

    fig, (ax, axa) = plt.subplots(1, 2, figsize=(11, 5))
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.plot(xs, ys, "--", color="gray", lw=1, label="target path")
    ax.add_patch(plt.Circle((0, 0), reach, fill=False, ls=":", color="tab:blue"))
    ax.add_patch(plt.Circle((0, 0), abs(L1 - L2), fill=False, ls=":", color="tab:blue"))
    arm, = ax.plot([], [], "o-", lw=5, color="black", ms=8)
    trail, = ax.plot([], [], color="tab:red", lw=2, label="end effector")
    ax.set_title("2-link arm following a path (dotted = workspace limits)")
    ax.legend(loc="upper right")

    steps = np.arange(len(xs))
    axa.plot(steps, deg[:, 0], label="joint 1")
    axa.plot(steps, deg[:, 1], label="joint 2")
    axa.set(xlabel="step", ylabel="angle (deg)", title="Joint angles from inverse kinematics")
    axa.grid(alpha=0.3)
    axa.legend()
    cursor = axa.axvline(0, color="gray")

    def update(i):
        (x1, y1), (x2, y2) = forward(*angles[i])
        arm.set_data([0, x1, x2], [0, y1, y2])
        trail.set_data(xs[: i + 1], ys[: i + 1])
        cursor.set_xdata([i, i])
        return arm, trail, cursor

    anim = FuncAnimation(fig, update, frames=len(xs), interval=30, blit=False)
    if save:
        anim.save(save, writer=PillowWriter(fps=30))
        print(f"Saved {save}")
    else:
        plt.show()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="2-link planar arm simulation")
    p.add_argument("--path", choices=["circle", "line"], default="circle")
    p.add_argument("--save", metavar="FILE.gif", help="save animation instead of showing it")
    args = p.parse_args()
    xs, ys = circle_path() if args.path == "circle" else line_path()
    animate(xs, ys, args.save)
