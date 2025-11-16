from abc import ABC, abstractmethod
import logging

class TelemetryHandlerBase(ABC):
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)

    @abstractmethod
    def get_serialized(self) -> dict:
        """Return the most recent telemetry data as a dictionary."""
        pass