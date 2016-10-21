#!/usr/bin/env python

"""
Fri Oct 21 20:09:07 EEST 2016, Nikos Koukis

Current node routes the multicast packets towards the wlan* interface so that
the current robot roscore can successfully find the other running agents'
roscores.

"""

from __future__ import print_function
import sys
import os
import subprocess
from subprocess import call

# Import the modules directory
modules_dir = os.path.join(os.path.dirname(__file__), "../misc")
if modules_dir not in sys.path:
    sys.path.insert(0, modules_dir)
from custom_exceptions import EnvironmentVarNotSetError, InterfaceNotFoundError

class_name = "multicast_rerouter"

def custom_print(msg):
    print(class_name + ": " + msg)

def main():
    # verify that the needed environment variables are set
    try:
        wlan_interface = os.environ["WLAN_INTERFACE"]
    except:
        raise EnvironmentVarNotSetError("WLAN_INTERFACE")

    # root access?
    if os.getuid() != 0:
        raise NoRootAccessError()


    # wlan interface exists?
    print("Checking if given interface is valid...")
    if call(["iwconfig", wlan_interface]) != 0:
        print("")
        raise InterfaceNotFoundError(wlan_interface)
    custom_print("OK")


    multicast_ip = "224.0.0.0"
    call(["route", "add", "-net", multicast_ip, "netmask",
                     multicast_ip, wlan_interface])


if __name__=="__main__":
    custom_print("Node is running")
    main();
