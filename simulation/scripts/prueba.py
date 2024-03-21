from sdl2 import *
import time
import pyautogui
import sys
import ctypes

class Joystick:
    def __init__(self):
        SDL_Init(SDL_INIT_JOYSTICK)
        self.device = None
        self.axis = {}
        self.button = {}
        self.centered = True
        self.left_click = False
        self.right_click = False
        self.middle_click = False

    def open(self, device_index):
        self.device = SDL_JoystickOpen(device_index)

    def update(self):
        event = SDL_Event()
        while SDL_PollEvent(ctypes.byref(event)) != 0:
            if event.type == SDL_JOYAXISMOTION:
                self.axis[event.jaxis.axis] = event.jaxis.value
                self.centered = False
            elif event.type == SDL_JOYBUTTONDOWN:
                self.button[event.jbutton.button] = True
                if event.jbutton.button == 0:  # Botón central (wheel)
                    self.middle_click = True
                    pyautogui.mouseDown(button='middle')
                elif event.jbutton.button == 1:  # Botón derecho
                    self.right_click = True
                    pyautogui.mouseDown(button='right')
                elif event.jbutton.button == 3:  # Botón izquierdo
                    self.left_click = True
                    pyautogui.mouseDown(button='left')
                elif event.jbutton.button == 10:  # El botón 11 en SDL2 es el 10 (los índices empiezan desde 0)
                    sys.exit()
            elif event.type == SDL_JOYBUTTONUP:
                self.button[event.jbutton.button] = False
                if event.jbutton.button == 0:  # Botón central (wheel)
                    self.middle_click = False
                    pyautogui.mouseUp(button='middle')
                elif event.jbutton.button == 1:  # Botón derecho
                    self.right_click = False
                    pyautogui.mouseUp(button='right')
                elif event.jbutton.button == 3:  # Botón izquierdo
                    self.left_click = False
                    pyautogui.mouseUp(button='left')

        if not any(self.axis.values()):
            self.centered = True

    def close(self):
        if self.device is not None:
            SDL_JoystickClose(self.device)
        SDL_Quit()

if __name__ == "__main__":
    joystick = Joystick()
    joystick.open(0)  # Cambiar el índice si tienes varios joysticks conectados

    screen_width, screen_height = pyautogui.size()

    sensitivity = 10  # Factor de sensibilidad más bajo = movimiento más lento

    while True:
        joystick.update()

        if joystick.centered:
            pyautogui.moveTo(screen_width // 2, screen_height // 2, duration=0.1)  # Volver al centro lentamente
        else:
            # Control del eje X
            mouse_x = pyautogui.position()[0]
            if 0 in joystick.axis:
                mouse_x += int(joystick.axis[0] / 32768 * sensitivity)

            # Control del eje Y
            mouse_y = pyautogui.position()[1]
            if 1 in joystick.axis:
                mouse_y += int(joystick.axis[1] / 32768 * sensitivity)

            pyautogui.moveTo(mouse_x, mouse_y, duration=0.0001)  # Mover el cursor del mouse más lentamente

        time.sleep(0.01)

    joystick.close()
