# Instructions for configuring the ad-hoc network
Tue Oct 4 11:31:01 EEST 2016, Nikos Koukis

Directory contains scripts that are used for the initial networking setup of
the agents in the multi-robot SLAM experiment. Users should run the
`seutp_adhoc.py` scirpt *having root privilages*. That registers an upstart job
at /etc/init/ directory which configures the corresponding interface on ad-hoc
mode on startup and when it is enabled.

<aside class="notice">
It is observed that the interface sometimes comes up with a different name to
the time it was configured. Ejecting and reinserting the adapter seems to fix
this problem
</aside>


