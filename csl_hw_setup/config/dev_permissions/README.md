When it comes to reading/writing to h/w devices on linux machines (e.g. laser, USB cameras, serial adapters etc) the corresponding application should have the necessary permissions to do so. Most times, that demands rw permissions on the `other` group.

To do that, a temporary solution would be to just grant these permissions like in the following situation:

`sudo chmod a+rw /dev/ttyACM0`

where ttyACM0 is an example terminal device.

However this setting is *not persistent* across computer reboots.

To make this persistent users should create a .rules file in /etc/udev/rules.d
directory with the necessary instructions for each h/w device they want to
modify permissions for. A sample configuration file for this purpose is given
in `mr_slam.rules`.
