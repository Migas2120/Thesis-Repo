# ros/commands/base_command.py

from abc import ABC, abstractmethod
import logging

class BaseCommand(ABC):
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)

    @abstractmethod
    def execute(self, ros, data: dict):
        pass
