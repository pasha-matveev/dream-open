from gpiozero import TonalBuzzer
from time import sleep


class Buzzer:
    def __init__(self, pin: int = 17) -> None:
        self.pin = pin
        self.tb = TonalBuzzer(pin)

    def play(self, note: str = 'C4', duration: float = 0.1) -> None:
        self.tb.play(note)
        sleep(duration)

    def play_tune(self, tune: list) -> None:
        for note, duration in tune:
            self.tb.play(note)
            sleep(duration)

    def stop(self) -> None:
        self.tb.stop()


if __name__ == '__main__':
    buzzer = Buzzer()
    tune = [('C#4', 0.2), ('D4', 0.2), (None, 0.2),
            ('Eb4', 0.2), ('E4', 0.2), (None, 0.6),
            ('F#4', 0.2), ('G4', 0.2), (None, 0.6),
            ('Eb4', 0.2), ('E4', 0.2), (None, 0.2),
            ('F#4', 0.2), ('G4', 0.2), (None, 0.2),
            ('C4', 0.2), ('B4', 0.2), (None, 0.2),
            ('F#4', 0.2), ('G4', 0.2), (None, 0.2),
            ('B4', 0.2), ('Bb4', 0.5), (None, 0.6),
            ('A4', 0.2), ('G4', 0.2), ('E4', 0.2),
            ('D4', 0.2), ('E4', 0.2)]
    buzzer.play_tune(tune)