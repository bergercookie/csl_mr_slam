#!/usr/bin/env bash

# Run this script in case internet access is needed for the agents running in
# the adhoc network. Script should be run **at the central node only** that
# already has internet access and needs to forward packets that originate from
# inside the ad-hoc towards the interface that connects to the world and
# vice-versa
#
# NOTE: Script should be executed only once in the central node. Afterwards, you
# should save the configuration and autoload it on system startup (e.g using
# the iptables-persistent package)

# make sure that we have root privilages
printf "Checking root access..."
if [[ "$(id -u)" == 0 ]]; then
    printf "OK\n"
else
    printf "\nUser doesn't have root access...\nExiting...\n"
    exit 1;
fi

printf "Input the ad-hoc interface: "
read ad_hoc_iface
printf "Input the interface that has internet access: "
read internet_iface
printf "Input the IP address domain (e.g. 10.8.0.0): "
read ip_domain

printf "Writing the firewall configuration...\n"
iptables \
    -A FORWARD -o ${internet_iface} -i ${ad_hoc_iface} \
    -s ${ip_domain} -m conntrack \
    --ctstate NEW -j ACCEPT
iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
iptables -t nat -F POSTROUTING
iptables -t nat -A POSTROUTING -o ${internet_iface} -j MASQUERADE

exit 0;
