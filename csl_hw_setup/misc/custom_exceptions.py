class CustomError(BaseException):
    """Class for implementing custom exceptions"""
    pass


class NoRootAccessError(CustomError):
    """Exception to be raised when the user doesn't have root access."""
    def __init__(self):
        pass
        
    def __str__(self):
        error_msg = "User doesn't have root access.\nExiting..."
        return error_msg

        
class ProgramNotFoundError(CustomError):
    """Exception to be raised when a program necessary for the script execution
    is not found."""

    def __init__(self, prog_name):
        self.prog_name = prog_name

    def __str__(self):
        error_msg = "".join([
            "Program {} is not in the user's path.".format(self.prog_name),
            "Either install it or modify the path.\n",
            "Exiting..."
        ])

        return error_msg


class InterfaceNotFoundError(CustomError):
    def __init__(self, interface_name):
        self.interface_name = interface_name


    def __str__(self):
        error_msg = ("Interface {} is not availabe.".format(
            self.interface_name))

        return error_msg

class EnvironmentVarNotSetError(CustomError):
    def __init__(self, var_name):
        self.var_name = var_name


    def __str__(self):
        error_msg = "".join([
            "Environment variable \"{}\" is not set. ".format(self.var_name),
            "Either specify it or consult the README.md file for available",
            "options on this\n",
            "Exiting"
        ])

        return error_msg
