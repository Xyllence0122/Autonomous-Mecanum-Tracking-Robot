#!/usr/bin/env python3
"""Print joystick axes and buttons for mapping."""

import time

import pygame


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick found")

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Joystick: {js.get_name()}")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 3) for i in range(js.get_numaxes())]
            buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
            print(f"axes={axes} buttons={buttons}", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()

