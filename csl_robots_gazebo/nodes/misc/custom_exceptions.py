class CustomError(BaseException):
    """Class for implementing custom exceptions"""
    pass


class RosLaunchSyntaxError(CustomError):
    def __init__(self, error_msg):
        self.error_msg = error_msg

    def __str__(self):
        return self.error_msg
