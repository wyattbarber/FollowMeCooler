from matplotlib import pyplot as plt
import serial
from typing import List, Tuple
import numpy as np

def decode(msg: str) -> List[Tuple[float, float]]:
    points = msg.split(';')[0]
    path_idx = int(msg.split(';')[1])
    pairs = points.split(',')
    data = [p.split(':') for p in pairs]
    angles = []
    dist = []
    for d in data:
        try:
            angles.append(float(d[0]))
            dist.append((float(d[1])))
        except ValueError:
            pass
    return (angles, dist, path_idx)


def update(fig, ax, x: List[float], y: List[float]):
    ax.scatter(x, y)
    fig.canv


def main():
    plt.ion()
    fig, ax = plt.subplots()
    line_scan, = ax.plot([], [], 'bo')
    line_path, = ax.plot([], [], 'r-')
    ax.set_xlim(-1000, 1000)
    ax.set_ylim(0, 1000)
    plt.draw()

    data_id = "Scanner Data:"
    with serial.Serial("COM9", 115200, timeout=None) as ser:
        while ser.is_open:
            msg = ser.read_until(b"\n")
            msg = msg.decode().strip('\r')
            if msg[:len(data_id)] == data_id:
                angles, dist, path_angle = decode(msg[len(data_id):])

                x = [p[1] * np.sin(np.deg2rad(p[0])) for p in zip(angles, dist)]
                y = [p[1] * np.cos(np.deg2rad(p[0])) for p in zip(angles, dist)]    

                line_scan.set_xdata(x)
                line_scan.set_ydata(y)

                line_path.set_xdata([0, 1000 * np.sin(np.deg2rad(path_angle))])
                line_path.set_ydata([0, 1000 * np.cos(np.deg2rad(path_angle))])
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                # print(path_angle)


if __name__ == "__main__":
    main()