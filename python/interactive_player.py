# import standard libraries
from abc import ABC, abstractmethod
import numpy as np
import cv2
import sys
import os

class InteractivePlayer(ABC):
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
