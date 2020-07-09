"""
QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision, Queensland University of Technology (QUT)
Brisbane, QLD 4000
All Rights Reserved

See LICENSE for the license information

Description: Interactive media player
Author: Lachlan Nicholson (Python)
"""

# import standard libraries
import os
import cv2
import sys
import numpy as np
from abc import ABC, abstractmethod

class InteractivePlayer(ABC):
    """ Abstract class which implements display loop and key handling. 
    Derived classes need only implement update() to draw their desired image,
    and handle_input(key) to modify player variables based off key input. """
    def __init__(self, player_name):
        self.player_name = player_name
        self.running = True

    def start(self):
        # create cv2 window
        cv2.namedWindow(self.player_name)

        # start window loop
        while self.running:

            # update window given state
            self.update()

            # get key press
            key = self.get_key()

            # use input to modify state
            self.handle_input(key)

            # clear buffer
            self.clear_key_buffer()

        # close cv2 windows
        cv2.destroyAllWindows()

    def get_key(self):
        key = cv2.waitKey(0)
        # while (key == -1):
        #     key = cv2.waitKey(1)
        if (key == -1):
            return key
        if (key == 27):
            self.running = False
        return chr(key)

    def clear_key_buffer(self):
        while (cv2.waitKey(1) != -1):
            pass

    @abstractmethod
    def update(self):
        
        raise NotImplementedError

    @abstractmethod
    def handle_input(self, key):
        raise NotImplementedError
