from gpiozero import Button


class Button:
    def __init__(self, pin: int) -> None:
        self.pin = pin
        self.button = Button(pin)

    def is_pressed(self) -> bool:
        return self.button.is_pressed

    def add_press_callback(self, callback):
        self.button.when_pressed = callback

    def add_release_callback(self, callback):
        self.button.when_released = callback


if __name__ == '__main__':
    buttons = [
        Button(17),
        Button(27),
        Button(22)
    ]

    while True:
        for button in buttons:
            if button.is_pressed():
                print('Button pressed')
            else:
                print('Button released')