#!/usr/bin/env python

"""
Tue Oct 4 12:31:34 EEST 2016, Nikos Koukis

Set up the adhoc network connection for the current host.

Current script generates an upstart service file, a priori provided a template
configuration file and based on the user-set interface, IP address parameters.

"""

from __future__ import print_function
import argparse
from subprocess import call
import os
import sys
from time import gmtime, strftime

import logging as lg
lg.basicConfig(level=lg.DEBUG)


# Import the modules directory
modules_dir = os.path.join(os.path.dirname(__file__), "../../misc")
if modules_dir not in sys.path:
    sys.path.insert(0, modules_dir)
from custom_exceptions import NoRootAccessError, \
    ProgramNotFoundError, InterfaceNotFoundError

# dictionary of help messages for the cmd line arguments
arg_defaults = {
    "loglevel": "INFO",
    "wlan_interface": "wlan0",
    "interactive": False,
    "dnsmasq": False,
    "dry_run": False,
    "ip_address": "10.8.0.1",
    "access_internet_via" : "",
}
help_msgs = {
    "loglevel": "Set the logging level of the current script",
    "wlan_interface": "Specify the wireless interface",
    "dry_run": "Write potential configuration contents to console only (dry run)",
    "ip_address": "Specify the IP address of the current host",
    "interactive": "Run script in interactive mode",
    "dnsmasq": ("Enable dnsmasq service on current host? "
                 "Use this option only in the computer that is to run as the "
                 "DNS provider"),
    "access_internet_via": ("Enable internet access to current host via another host "
                            "in the common ad-hoc network. "
                            "Specify the address of that link")
}

def format_help_msg(cmd_var):
    """Get the help message in a formatted version.

    Function takes in account the default value and initial help message set
    for the given variable name
    """

    assert cmd_var in help_msgs.keys()
    help_msg = help_msgs[cmd_var]

    # append a string indicating the default value
    if cmd_var in arg_defaults.keys():
        default_str = " [default: {}]".format(arg_defaults[cmd_var])

        if help_msg.endswith(": "):
            help_msg = help_msg[:-2] + default_str + ": "
        else:
            help_msg = help_msg + default_str

    return help_msg


# Arguement Parsing
parser = argparse.ArgumentParser(
    description='Setup Ad-Hoc network configuration for multi-robot SLAM')

# logging level
arg_name = "loglevel"
parser.add_argument(
    '-l', '--{}'.format(arg_name),
    default=arg_defaults[arg_name],
    help=format_help_msg(arg_name))

# wireless interface
arg_name = "wlan_interface"
parser.add_argument(
    '-w', '--{}'.format(arg_name),
    default=arg_defaults[arg_name],
    help=format_help_msg(arg_name))

# dry-run
arg_name = "dry_run"
parser.add_argument(
    '-d', '--{}'.format(arg_name),
    action="store_true",
    help=format_help_msg(arg_name))

# ip-address of current host
arg_name = "ip_address"
parser.add_argument(
    '-I', '--{}'.format(arg_name),
    default=arg_defaults[arg_name],
    help=format_help_msg(arg_name))

# Run interactively
arg_name = "interactive"
parser.add_argument(
    "-i", "--{}".format(arg_name),
    default=arg_defaults[arg_name],
    action="store_true",
    help=format_help_msg(arg_name))

# Mutually exclusive, internet-related arguments
internet_related = parser.add_mutually_exclusive_group(required=False)

arg_name = "dnsmasq"
internet_related.add_argument(
    "-m", "--{}".format(arg_name),
    action="store_true", # defaults to opposite
    help=format_help_msg(arg_name))

arg_name = "access_internet_via"
internet_related.add_argument(
    "-a", "--{}".format(arg_name),
    default=arg_defaults[arg_name],
    help=format_help_msg(arg_name))

# Strings to their boolean correspondence
str_to_bool = {
    "Yes": True,
    "True": True,
    "1": True,
    "Yup": True,
    True: True,

    "No": False,
    "False": False,
    "0": False,
    "Nope": False,
    False: False
}


parser_args = vars(parser.parse_args())

def raw_input_wrapper(help_msg):
    """Wrapper around the raw_input function."""

    str_append = ": "
    if not help_msg.endswith(str_append):
        help_msg += str_append

    ret = raw_input(help_msg)
    return ret


def raw_input_wrapper_cmd_var(cmd_var, cmd_var_type=str):
    """
    Wrapper around the raw_input_wrapper function.

    Takes the name of an argparse cmd variable instead and uses that to fetch
    the corresponding help message

    :param cmd_var: Name of variable
    :type cmd_var: str

    :param cmd_var_type: Type of input variable
    :type cmd_var_type: Python type
    """

    help_msg = format_help_msg(cmd_var)
    if cmd_var in arg_defaults.keys():
        default_val = arg_defaults[cmd_var]
    else:
        default_val = ""

    ret = raw_input_wrapper(help_msg)
    if ret is "":
        ret = default_val

    # input validation
    if cmd_var_type == bool and isinstance(ret, str):
        if not ret in str_to_bool.keys():
            raise ValueError('Invalid value of boolean %s argument: %s' %(cmd_var, ret))
        else:
            ret = str_to_bool[ret]

        ret = str_to_bool[ret]
    if not isinstance(ret, cmd_var_type):
        raise ValueError('Invalid value of %s argument: %s' %(cmd_var, ret))

    return ret



def main():
    """Main."""

    debug_file = "/var/log/multi_robot_exp_status"

    # fetch the arguments
    interactive = parser_args["interactive"]
    # with no arguments given, run in interactive
    if len(sys.argv) == 1:
        interactive = True

    if interactive:
        lg.warn("In interactive mode:")
        wlan_interface = raw_input_wrapper_cmd_var("wlan_interface")
        ip_address = raw_input_wrapper_cmd_var("ip_address")
        dnsmasq = raw_input_wrapper_cmd_var("dnsmasq", bool)
        access_internet_via = raw_input_wrapper_cmd_var("access_internet_via")
        dry_run = raw_input_wrapper_cmd_var("dry_run", bool)
        loglevel = raw_input_wrapper_cmd_var("loglevel")

        # only one of dnsmasq and access_internet_via should be specified
        assert bool(dnsmasq) != bool(access_internet_via) and \
            ("Both dnsmasq and access_internet_via arguments have been set."
             "Set exclusively one of them and rerun")
    else: # fetch from argparse
        wlan_interface = parser_args["wlan_interface"]
        ip_address = parser_args["ip_address"]
        dnsmasq = parser_args["dnsmasq"]
        access_internet_via = parser_args["access_internet_via"]
        dry_run = parser_args["dry_run"]
        loglevel = parser_args["loglevel"]

    # validate the loglevel input
    numeric_level = getattr(lg, loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    lg.basicConfig(level=numeric_level)

    # Specify whether we are running for real or on a dry-run
    if dry_run:
        lg.warn("Script is running on a dry run! Specified commands will not be executed")

    check_reqs(ip_address=ip_address, wlan_interface=wlan_interface, dry_run=dry_run)

    # check if upstart file already exists
    upstart_fname = "setup_adhoc.conf"
    upstart_fname_full = os.path.join("/etc/init/", upstart_fname)
    if not dry_run:
        assert\
            not os.path.isfile("/etc/init/setup_adhoc.conf") and\
            "/etc/init/setup_adhoc.conf already exists. "\
            "Remove it in case you want the script to write it again."

    # read template configuration file
    script_dir = os.path.dirname(os.path.realpath(__file__))
    template_conts = open(os.path.join(script_dir, "setup_adhoc.conf.template"), 'r').readlines()

    if dnsmasq: # add line to restart dnsmasq
        lg.info("dnsmasq is to be restarted.")
        dnsmasq_line = "service dnsmasq restart"
        template_conts.insert(-3, dnsmasq_line)
    elif access_internet_via: # add the internet-access related contents
        internet_template_conts = open(
            os.path.join(
                script_dir, "access_internet.conf.template"), 'r').readlines()
        for line in internet_template_conts:
            template_conts.insert(-3, line)

    # write the correct wlan interface and ip address
    # ignore comment lines
    fun = lambda l: l if l.startswith("#") else l.\
        replace("IP_ADDRESS", ip_address).\
        replace("WLAN_INTERFACE", wlan_interface).\
        replace("INTERNET_LINK", access_internet_via).\
        replace("DEBUG_FILE", debug_file)

    template_conts_modified = map(fun, template_conts)

    if not dry_run:
        lg.info("Writing upstart job...")
        with open(upstart_fname_full, "w") as upstart_f:
            upstart_f.writelines(template_conts_modified)
        lg.info("Successfully written ad-hoc configuration to upstart job: %s", upstart_fname_full)
    else:
        header = "Contents of file to be written (dry run):\n"
        header += "="*40
        lg.info(header)
        for line in template_conts_modified:
            print(line)

    lg.info("Exiting...")

def check_reqs(**kargs):
    """Check if the reuired tools exist in the system."""

    # root access?
    if os.getuid() != 0 and not kargs["dry_run"]:
        raise NoRootAccessError()

    # ifconfig installed?
    lg.info("Checking if ifconfig command is available...")
    if call(["which", "ifconfig"]) != 0:
        raise ProgramNotFoundError("ifconfig")
    lg.info("OK")

    # wlan interface exists?
    lg.info("Checking if given interface is valid...")
    if call(["iwconfig", kargs["wlan_interface"]]) != 0:
        print("")
        raise InterfaceNotFoundError(kargs["wlan_interface"])
    lg.info("OK")



if __name__ == "__main__":
    main()

