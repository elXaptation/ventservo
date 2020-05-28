#!/bin/bash
# Run as root/sudo
groupadd gpio
usermod -a -G gpio $1
