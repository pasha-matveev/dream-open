from gpiozero import Button as GButton


class Button:
    def __init__(self, pin: int) -> None:
        self.pin = pin
        self.button = GButton(pin)
        self.pressed = False
        self.first_pressed = False

    def read(self):
        if self.pressed:
            if self.button.is_active:
                self.pressed = True
                self.first_pressed = False
            else:
                self.pressed = False
                self.first_pressed = False
        else:
            if self.button.is_active:
                self.pressed = True
                self.first_pressed = True
            else:
                self.pressed = False
                self.first_pressed = False


class ButtonArray:
    def __init__(self, pins: list) -> None:
        self.pins = pins
        self.buttons = [Button(pin) for pin in pins]

    def read(self):
        return [button.read() for button in self.buttons]

    def __getattr__(self, name):
        return self.buttons[name].first_pressed


if __name__ == "__main__":
    buttons = ButtonArray([17, 27, 22])

    while True:
        buttons.read()
        for i, but in enumerate(buttons.buttons):
            if but.pressed:
                print(f"Button {i + 1} is pressed")
