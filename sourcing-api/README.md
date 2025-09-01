Minimal container for recording
===============================

This is a minimal container for sourcing. It runs with a single Ouster and a
single imaging source camera.

The sourcing kit is implemented for ARM64v8 architectures. Tests were conducted
on ML-Joey.

Ouster configuration:
 - 2048x1028
 - 10 Hz
 - Network interface: enx00133bfcbcea
 - IP: 169.254.65.37

Imaging source configuration:
 - 2448x2048
 - 10 Hz
 - Network interface: eno1
 - IP: 169.254.55.255


Network configuration
---------------------

Use a link-local network connection to connect to the system. Don't forget to
activate it.

Run

    ping ML-Joey.local

from your machine to figure out the IP address of the sourcing machine.


Connection
----------

You have to SSH into ML-Joey to run the sourcing. Run

    ssh sourcing@ML-Joey.local -X

to connect to the server.


Debugging
---------

If the sensors are not visible or you encounter a high packet los, running

    scripts/host/routes.bash

with `sudu` from the sourcing machine should already solve many issues. The
script sets the routes for the sensors and also increases the MTU of the
Imaging Source camera.

If one of the sensors is not pingable, it makes sense to set a route to this
sensor. E.g.

    route add -host 169.254.55.225 dev eno1

If the IP address of the Imaging Source sensor is unknown, try

    arv-tool-0.8 features

If the Aravis ROS2 driver yields an error like

    Frame error: ARV_BUFFER_STATUS_TIMEOUT

then it might be sensible to increase the MTU of the imaging source network
interface:

    sudo ifconfig eno1 mtu 9000

If no camera is found, try

    sudo pktstat -n -i <INTERFACE>


PTP
---

PTP traffic might be blocked. Try the following lines to allow traffic on
PTP ports 123 and 3265 (UDP):

    iptables -A INPUT -p udp --dport 123 -j ACCEPT
    iptables -A OUTPUT -p udp --sport 123 -j ACCEPT

    iptables -A INPUT -p udp --dport 3265 -j ACCEPT
    iptables -A OUTPUT -p udp --sport 3265 -j ACCEPT

    sudo phc2sys -w -s eno1 -O 0 -m
    sudo phc2sys -a -r -m
    sudo ptp4l -i eno1 -l 7 -q -m -H -p /dev/ptp0
