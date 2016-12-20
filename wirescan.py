import numpy as np

# units are mm
example_range = [(14.160, 18.160),  # u
                 (28.810, 32.810),  # x
                 (38.320, 42.320),  # y
                 ]
# distance scanned over the wire
scan_dist = 4.000
# number of points desired from the scientists' standpoint
desired_points = 100
# max speed the stage can move
max_speed = 100  # 300


def running_mean(x, n):
    return np.convolve(x, np.ones((n,)) / n)[(n - 1):]


def get_speed(rep_rate):
    return min(max_speed, rep_rate * scan_dist / desired_points)


def build_velocity_profile(speed, max_speed, dt=0.001, scan_ranges=None):
    if scan_ranges is None:
        scan_ranges = example_range

    distances = [
                 # wire u
                 scan_ranges[0][0],
                 scan_dist,

                 # wire x
                 scan_ranges[1][0] - scan_ranges[0][1],
                 scan_dist,

                 # wire y
                 scan_ranges[2][0] - scan_ranges[1][1],
                 scan_dist,

                 # return (using negative speed below)
                 scan_ranges[2][1]
                 ]

    speeds = [max_speed, speed,
              max_speed, speed,
              max_speed, speed,
              -max_speed
              ]

    vel_profile = [0] * 10
    for speed, distance in zip(speeds, distances):
        dt0 = abs(distance / speed)
        vel_profile += [speed] * int(dt0 / dt)  # not exact

    return vel_profile


def build_position_profile(dt, vel_profile, smoothing=3):
    x_time = np.arange(0.0, len(vel_profile) * dt, dt)
    pos_profile = np.cumsum(np.array(vel_profile) * dt)

    # TODO
    i = len(pos_profile) - 1
    while pos_profile[i] < 0:
        pos_profile[i] = 0
        i -= 1

    pos_profile = running_mean(pos_profile, smoothing)

    return (x_time, pos_profile)


def _test_plot(speed, max_speed, dt, smoothing):
    import matplotlib
    matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt

    vel_profile = build_velocity_profile(speed, max_speed, dt=dt)
    x_time, pos_profile = build_position_profile(dt, vel_profile,
                                                 smoothing=smoothing
                                                 )

    print(len(pos_profile), len(vel_profile))
    plt.plot(x_time, pos_profile)
    plt.title('speed={} max_speed={} dt={} smoothing={}\nnum_points={}'
              ''.format(speed, max_speed, dt, smoothing, len(x_time)))

    ax2 = plt.twinx()
    ax2.plot(x_time, vel_profile, '--r')
    plt.show()


if __name__ == '__main__':
    speed = get_speed(rep_rate=100)
    _test_plot(speed, max_speed, dt=0.01, smoothing=3)
