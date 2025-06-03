from typing import Optional
from abc import abstractmethod, ABC


class Robot(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def move(self):
        pass