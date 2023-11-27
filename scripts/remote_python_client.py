#!/usr/bin/env python

"""@package dynamic_graph_bridge.

@file
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.

@brief Instanciate a python terminal communication with the dynamic-graph
embeded terminal.

"""

import atexit
import code

# Standard import.
import optparse
import os
import os.path
import readline
import signal
import sys
from pathlib import Path

import rclpy

# Used to connect to ROS services
from dynamic_graph_bridge.ros.dgcompleter import DGCompleter
from dynamic_graph_bridge_cpp_bindings import RosPythonInterpreterClient


def signal_handler(sig, frame):
    """Catch Ctrl+C and quit."""
    print("")
    print("You pressed Ctrl+C! Closing ros client and shell.")
    rclpy.try_shutdown()
    sys.exit(0)


# Command history, auto-completetion and keyboard management
python_history = Path.join(os.environ["HOME"], ".dg_python_history")
readline.parse_and_bind("tab: complete")
readline.set_history_length(100000)


def save_history(histfile):
    """Write the history of the user command in a file."""
    readline.write_history_file(histfile)


"""
Read the current history if it exists and program
its save upon the program end.
"""
if hasattr(readline, "read_history_file"):
    try:
        readline.read_history_file(python_history)
    except OSError:
        pass
    atexit.register(save_history, python_history)


class DynamicGraphInteractiveConsole(code.InteractiveConsole):
    """For the subtilities please read https://docs.python.org/3/library/code.html."""

    def __init__(self):
        """Create interpreter in ROS for DG interactive console."""
        # Create the python terminal
        code.InteractiveConsole.__init__(self)

        # Command lines from the terminal.
        self.lines_pushed = ""

        self.ros_python_interpreter = RosPythonInterpreterClient()
        if sys.version[:2].startswith("2."):
            self.dg_completer = DGCompleter(self.ros_python_interpreter)
            readline.set_completer(self.dg_completer.complete)

    def runcode(self, code):
        """Inherited from code.InteractiveConsole.

        We execute the code pushed in the cache `self.lines_pushed`.
        The code is pushed whenever the user press enter during the
        interactive session.
        see https://docs.python.org/3/library/code.html
        """
        try:
            # we copy the line in a tmp var
            code_string = self.lines_pushed[:-1]
            rpi = self.ros_python_interpreter
            result = rpi.run_python_command(code_string)

            self.write(result)
            if not result.endswith("\n"):
                self.write("\n")
            # we reset the cache here
            self.lines_pushed = ""
        except Exception as e:
            self.write(str(e))
            return False

    def runsource(self, source, filename="<input>", symbol="single"):
        """Inherited from code.InteractiveConsole.

        see https://docs.python.org/3/library/code.html
        """
        try:
            c = code.compile_command(source, filename, symbol)
            if c:
                return self.runcode(c)
            return True
        except SyntaxError:
            self.showsyntaxerror()
            self.lines_pushed = ""
            return False
        except OverflowError:
            self.showsyntaxerror()
            self.write("OverflowError")
            self.lines_pushed = ""
            return False

    def push(self, line):
        """Upon pressing enter in the interactive shell the user "push" a string."""
        """This method is then called with the string pushed.
        We catch the string to send it via the rosservice."""
        self.lines_pushed += line + "\n"
        return code.InteractiveConsole.push(self, line)


if __name__ == "__main__":
    rclpy.init()

    parser = optparse.OptionParser(usage="\n\t%prog [options]")
    (options, args) = parser.parse_args(sys.argv[1:])

    dg_console = DynamicGraphInteractiveConsole()

    if args[:]:
        infile = args[0]
        rpi = dg_console.ros_python_interpreter
        response = rpi.run_python_script(Path.abspath(infile))
        print(rpi.run_python_command("print('File parsed')"))

    signal.signal(signal.SIGINT, signal_handler)
    dg_console.interact("Interacting with remote python server.")
    rclpy.shutdown()
