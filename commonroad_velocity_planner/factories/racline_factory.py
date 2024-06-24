import abc
from abc import ABC, abstractmethod



class BaseRaceLineCreator(ABC):

    @staticmethod
    @abstractmethod
    def factory_method():
        """
        Abstract factory method for RaceLine Factories
        """
        pass



