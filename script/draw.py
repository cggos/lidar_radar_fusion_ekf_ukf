import numpy as np
import matplotlib.pyplot as plt


def main():
    of = open("../build/output.txt")

    est_xs = []
    est_ys = []

    meas_xs = []
    meas_ys = []

    true_xs = []
    true_ys = []

    for line in of:
        data = line.split()
        assert len(data) == 10

        est_x = float(data[0])
        est_y = float(data[1])
        est_vx = float(data[2])
        est_vy = float(data[3])

        meas_x = float(data[4])
        meas_y = float(data[5])

        true_x = float(data[6])
        true_y = float(data[7])
        true_vx = float(data[8])
        true_vy = float(data[9])

        est_xs.append(est_x)
        est_ys.append(est_y)

        meas_xs.append(meas_x)
        meas_ys.append(meas_y)

        true_xs.append(true_x)
        true_ys.append(true_y)

    plt.plot(meas_xs, meas_ys, 'bo', label='Measurement')
    plt.plot(est_xs, est_ys, 'g-', label='Estimation')
    plt.plot(true_xs, true_ys, 'r-', label='Groundtruth')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
