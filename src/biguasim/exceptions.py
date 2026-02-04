"""BiguaSim Exceptions"""


class BiguaSimException(Exception):
    """Base class for a generic exception in BiguaSim.

    Args:
        message (str): The error string.
    """


class BiguaSimConfigurationException(BiguaSimException):
    """The user provided an invalid configuration for BiguaSim"""


class TimeoutException(BiguaSimException):
    """Exception raised when communicating with the engine timed out."""


class NotFoundException(BiguaSimException):
    """Raised when a package cannot be found"""

