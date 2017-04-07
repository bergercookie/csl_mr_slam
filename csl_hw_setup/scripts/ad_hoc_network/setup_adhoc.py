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


# Import the modules directory
modules_dir = os.path.join(os.path.dirname(__file__), "../../misc")
if modules_dir not in sys.path:
    sys.path.insert(0, modules_dir)
from custom_exceptions import NoRootAccessError, \
    ProgramNotFoundError, InterfaceNotFoundError

wlan_interface_default="wlan0"
ip_address_default="10.8.0.1"

# Arguement Parsing
parser = argparse.ArgumentParser(
    description='Setup Ad-Hoc network configuration for multi-robot SLAM')

# wireless interface
parser.add_argument(
    '-w', '--wlan_interface',
    default=wlan_interface_default,
    help='Specify the wireless interface [default = {}]'.format(wlan_interface_default))

# dry-run flag
parser.add_argument(
    '-d', '--dry-run',
    default=False,
    action="store_true",
    help="Monitor the actions that the script will take if executed")

# ip-address of current host
parser.add_argument(
    '-I', '--ip_address',
    default=ip_address_default,
    help="Specify the IP address of the current host [default = {}]".format(ip_address_default))

# Run interactively
parser.add_argument(
    "-i", "--interactive",
    action="store_true",
    help="Run script in interactive mode")
args = vars(parser.parse_args())


def main():
    """Main."""

    interactive = args["interactive"]
    dry_run = args["dry_run"]

    # Specify whether we are running for real or on a dry-run
    if dry_run:
        print("[!] Script is running on a dry run! Specified commands will not be executed")

    # at least one argument should be given
    if len(sys.argv) == 1:
        print(
            "[!] At least one argument shoould be given. Specify \"--help\" to get a list of arguments\n",
            "Exiting...")
        sys.exit(1)


    if interactive:
        print("[!] In interactive mode:")
        wlan_interface = raw_input(
            "wlan interface to be used: [Default = {}] ".format(wlan_interface_default))
        ip_address = raw_input(
            "Current host's IP address in ad-hoc: [Default = {}] ".format(ip_address_default))
    else:
        wlan_interface = args["wlan_interface"]
        ip_address = args["ip_address"]

    check_reqs(ip_address=ip_address, wlan_interface=wlan_interface)

    # check if upstart file already exists
    upstart_fname = "setup_adhoc.conf"
    upstart_fname_full = os.path.join("/etc/init/", upstart_fname)
    if not dry_run:
        assert\
            not os.path.isfile("/etc/init/setup_adhoc.conf") and\
            "/etc/init/setup_adhoc.conf already exists. Remove it in case you want the script to write it again."

    # read template configuration file
    script_dir = os.path.dirname(os.path.realpath(__file__))
    template_conts = open(os.path.join(script_dir, "setup_adhoc.conf.template"), 'r').readlines()

    # write the correct wlan interface and ip address
    # ignore comment lines
    fun = lambda l: l if l.startswith("#") else l.\
        replace("IP_ADDRESS", ip_address).\
        replace("WLAN_INTERFACE", wlan_interface)

    template_conts_modified = map(fun, template_conts)

    if not dry_run:
        print("Writing upstart job...")
        with open(upstart_fname_full, "w") as upstart_f:
            upstart_f.writelines(template_conts_modified)
        print("Successfully written ad-hoc configuration to upstart job: {}".format(
            upstart_fname_full))
    else:
        header = "Contents of file to be written (dry run):\n"
        header += "="*40
        print(header)
        for line in template_conts_modified:
            print(line)

    print("Exiting...")

def check_reqs(**kargs):
    """Check if the reuired tools exist in the system."""

    # root access?
    if os.getuid() != 0:
        raise NoRootAccessError()

    # ifconfig installed?
    print("Checking if ifconfig command is available...")
    if call(["which", "ifconfig"]) != 0:
        raise ProgramNotFoundError("ifconfig")
    print("OK")

    # wlan interface exists?
    print("Checking if given interface is valid...")
    if call(["iwconfig", kargs["wlan_interface"]]) != 0:
        print("")
        raise InterfaceNotFoundError(kargs["wlan_interface"])
    print("OK")



if __name__ == "__main__":
    main()

