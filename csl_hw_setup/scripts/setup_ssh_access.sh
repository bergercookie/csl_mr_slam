#!/usr/bin/env bash
# Wed Apr 19 10:35:39 EEST 2017, Nikos Koukis

# Run this script from the central node in a multi-robot SLAM experiment to
# have the SSH public key of the central node registered in all hosts for easy
# and secure remote access

# Make sure that ~/.ssh_id_rsa.pub exists
ssh_key_path="${HOME}/.ssh/id_rsa.pub"
printf "Checking that own public key exists..."
if ! [[ -f ${ssh_key_path} ]]; then
    printf "\nPublic key not there (%s).\nPlease create a private/public key pair and rerun script\n" ${ssh_key_path}
    exit 1;
else
    printf "Done.\n"
fi



printf "Input IP address of agent: "
read ssh_addr
printf "Select the username for remote agent \"%s\": " "${ssh_addr}"
read ssh_user
ssh_remote_full="${ssh_user}@${ssh_addr}"

#echo "Specify password for \"${ssh_remote_full}\":"
#read -s ssh_passwd

printf "Initiating connection to \"%s\"...\n" "${ssh_remote_full}"

cat ${ssh_key_path} | ssh ${ssh_remote_full} 'umask 0077; mkdir -p .ssh; cat >> .ssh/authorized_keys'
echo "Key copied!"
exit 0;
