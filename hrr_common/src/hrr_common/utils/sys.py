#!/usr/bin/env python3
"""
Python system setup helper utilities
-----------------------------------------

This module contains a collection of helpful system / os-relevant helper utilities
"""

import socket
import os

__all__ = ["get_hostname_and_IP", "set_ros_environment", "in_tum_lab"]


def in_tum_lab():
    """check if current PC is in TUM-Lab"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return "129.187" in s.getsockname()[0]


def get_hostname_and_IP():
    """
    Get hostname and IP of current PC.

    This function simply pings google-DNS server via http
    to force the usage of the network IP and not arbitrary
    network interfaces such as the internal localhost (127.0.0.1)
    or p2p-network with e.g. hardware-clients

    Returns:
        tuple(str, str): hostname and IP of current client
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return socket.gethostname(), s.getsockname()[0]


def print_ros_setup(host, ip, ros_master, ros_ip):
    """
    Print current ROS / network configuration in human-readable manner
    Prints warning in case current ROS-IP is different from actual IP

    Args:
        host(str): current hostname
        ip(str): current IP
        ros_master(str): ROS-master-URI, i.e. hostname / IP and port
        ros_ip(str): current ROS-IP
    """
    if ip != ros_ip:
        print(f"\t\033[1m\033[4m!!!your IP: {ip} is different from your ROS_IP{ros_ip}!!!\033[0m")
    print(f"current hostname:\t{host}\n"
          f"current IP:      \t{ip}\n"
          f"ROS-MASTER-URI:  \t{ros_master}")


def set_ros_environment(ros_master, print_setup=True, port=11311):
    """
    Set and print current ROS-environment.
    Get current system network via :py:func:`~get_hostname_and_IP`.
    Eventually print information via :py:func:`~print_ros_setup`.

    Args:
        ros_master(str or None): ros-master IP or hostname or blank (None), which equals to current PC
        print_setup(bool, optional): flag to print system setup to terminal. Defaults to True.
        port(int, optional): TCP-port for ROS-master. Defaults to ROS-default, i.e. 11311.
    """
    host, ip = get_hostname_and_IP()
    ros_ip = os.getenv("ROS_IP")
    if ros_ip != ip:
        os.environ["ROS_IP"] = ip
    if ros_master in ('localhost', '127.0.0.1', None):
        ros_master = '127.0.0.1'
    os.environ["ROS_MASTER_URI"] = f"http://{ros_master}:{port}"
    if print_setup:
        print_ros_setup(host, ip, os.getenv("ROS_MASTER_URI"), os.getenv("ROS_IP"))

