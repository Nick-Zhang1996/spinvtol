import pygame

if __name__=="__main__":
    pygame.display.init()
    pygame.joystick.init()
    try:
        js = pygame.joystick.Joystick(0)
        js.init()
        num_axes = js.get_numaxes()
        while True:
            pygame.event.pump()
            # 5 is right trigger, -1.0 -> 1(fully depressed)
            # 1 is left joystick up/down channel, up -1, down 1
            print([js.get_axis(j) for j in range(num_axes)])
    except pygame.error as e:
        print(e)

