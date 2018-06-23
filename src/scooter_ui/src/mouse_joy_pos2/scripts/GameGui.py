#! /usr/bin/env python

#####
##
##
##
#####

import pygame

class GameGui():
    def __init__(self):
        self.backgroundColor = 0, 0, 0
        self.blueval = 0
        self.bluedir = 1
        self.windowOffset = 100
        self.windowX = 500
        self.windowY = 500
        self.screen = pygame.display.set_mode((self.windowX, self.windowY))
    
    
    def update(self, x, y):
        self.screen.fill(self.backgroundColor)
        pygame.draw.line(self.screen, (0, 0, self.blueval), (x, 0), (x, 399))
        pygame.draw.line(self.screen, (0, 0, self.blueval), (0, y), (639, y))
        self.blueval += self.bluedir
        if self.blueval == 255 or self.blueval == 0:
            self.bluedir *= -1
        pygame.display.flip()
