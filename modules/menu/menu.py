import pygame
from modules.menu.display import Display

# on/off motors
# auto/on/off dribbling
# on/off kicker
# force kick
#


class Page:
    def __init__(self, name, action, update):
        self.name = name
        self.action = action
        self.update = update


class Menu:
    def __init__(self, robot):
        self.robot = robot

        self.open_page = None
        self.page_list = [
            Page()
        ]

        self.current_page_index = 0
        self.first_page_index = 0
        self.pages_count = 3

    def update(self):
        btn = [False, False, False]

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.robot.running = False
                elif event.key == pygame.K_UP:
                    btn[0] = True
                elif event.key == pygame.K_DOWN:
                    btn[1] = True
                elif event.key == pygame.K_SPACE:
                    btn[2] = True
            if event.type == pygame.QUIT:
                self.robot.running = False

        if self.open_page == None:
            if btn[2]:
                self.open_page = self.page_list[self.current_page_index]
                self.open_page.action()
            else:
                if btn[0]:
                    self.current_page_index = max(self.first_page_index - 1, 0)
                    self.first_page_index = min(self.current_page_index, self.first_page_index)
                elif btn[1]:
                    self.current_page_index = min(self.current_page_index + 1, len(self.page_list) - 1)
                    self.first_page_index = max(self.first_page_index, self.current_page_index - self.pages_count + 1)
