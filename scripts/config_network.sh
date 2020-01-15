#!/usr/bin/env bash

ip addr add 192.168.1.42/24 dev ens20u2
ip link set ens20u2 up

# ping 192.168.1.204