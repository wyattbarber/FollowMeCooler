from matplotlib import pyplot as plt
import serial
from typing import List, Tuple

def decode(msg: str) -> List[Tuple[float, float]]:
    pairs = msg.split(',')
    data = [p.split(':') for p in pairs]
    out = []
    for d in data:
        try:
            out.append((float(d[0]), float(d[1])))
        except ValueError:
            pass
    return out


def update(fig, ax, x: List[float], y: List[float]):
    ax.scatter(x, y)
    fig.canv


def main():
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlim(-100, 100)
    ax.set_ylim(0, 100)
    plt.draw()
    with serial.Serial("COM4", 115200, timeout=None) as ser:
        while ser.is_open:
            msg = ser.read_until(b"\n")
            pts = decode(msg.decode().strip('\r'))
            print("Recieved an update!\n")
            for p in pts:
                print(f"\t({p[0]}, {p[1]})\n")

            line.set_xdata([p[0] for p in pts])
            line.set_ydata([p[1] for p in pts])
            
            fig.canvas.draw()
            fig.canvas.flush_events()


if __name__ == "__main__":
    main()