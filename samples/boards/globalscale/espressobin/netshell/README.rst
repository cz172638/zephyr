.. zephyr:code-sample:: espressobin-netshell
   :name: ESPRESSObin network shell
   :relevant-api: ssh_server

   Headless ESPRESSObin: DHCP, SSH shell and firmware update over TCP.

Overview
********

For a board without a working serial console. On boot the sample:

* keeps a random MAC address stable across reboots (settings in the last
  64 KiB of the SPI NOR, the old U-Boot environment),
* gets an address with DHCP and announces ``espressobin<mac>.local`` over
  mDNS (and the same name in the DHCP request),
* generates an RSA host key on the first boot and keeps it in settings,
* starts the SSH server with the Zephyr shell, allowing a built-in public
  key and/or a password.

From the shell, ``fwupdate tcp`` replaces the SPI NOR boot image.

Building
********

Zephyr's SSH server accepts RSA keys. Export the public key in PEM/PKCS8:

.. code-block:: console

   ssh-keygen -e -m PKCS8 -f ~/.ssh/id_rsa.pub > authorized_key.pem

.. zephyr-app-commands::
   :zephyr-app: samples/boards/globalscale/espressobin/netshell
   :board: espressobin/armada3720/smp
   :gen-args: -DNETSHELL_AUTHORIZED_KEY=authorized_key.pem
   :goals: build

Without a key, set ``CONFIG_NETSHELL_PASSWORD``; it is stored in clear
text in the image. The first boot spends a few seconds generating the host
key.

With the ``espressobin-dsa`` snippet (``-S espressobin-dsa``) every front
port is its own interface (``swp1`` to ``swp3`` = switch ports 1 to 3) and
gets DHCP and a stable MAC address; ``eth0`` only carries the tagged
frames. Adding ``-DEXTRA_CONF_FILE=gptp.conf`` runs gPTP on the three ports
with the switch's time stamps (``net gptp``).

Router
======

.. code-block:: console

   west build -b espressobin/armada3720/smp -S espressobin-dsa -S espressobin-dsa-v7 \
      samples/boards/globalscale/espressobin/netshell -- -DEXTRA_CONF_FILE=router.conf

The second snippet names the ports after the front labels
(``espressobin-dsa-v5`` for v5 and older). ``wan`` gets an address with
DHCP. ``lan0`` and ``lan1`` join ``bridge0``: the switch forwards between
them itself, the bridge gets ``CONFIG_NETSHELL_LAN_ADDR`` (192.168.10.1/24)
and runs a DHCP server from ``CONFIG_NETSHELL_DHCP_POOL``, handing itself
out as the router. IPv4 is forwarded between ``wan`` and the LAN without
NAT, so the upstream router needs a route to the LAN, e.g. on Linux:

.. code-block:: console

   ip route add 192.168.10.0/24 via <wan address of the board>

``net bridge`` and ``net route`` show the state. gPTP on the three ports
combines with it: ``-DEXTRA_CONF_FILE="router.conf;gptp.conf"``.

Saved configuration
===================

Changes made from the shell apply at once but last only until a reboot,
as on network gear, until they are saved:

.. code-block:: console

   uart:~$ router lan 192.168.20.1/24     # router: LAN address, pool moves along
   uart:~$ router pool 192.168.20.50
   uart:~$ router dns 192.168.200.1       # or "auto": DNS server of the wan lease
   uart:~$ mv88e6xxx tcam add 1 10 lan0,lan1 drop proto icmp
   uart:~$ mvneta qav 4 3 20000000
   uart:~$ config save                    # LAN, TCAM rules, Qav of all interfaces
   uart:~$ config show
   uart:~$ config erase                   # defaults on the next boot

A rule that locks you out is therefore gone after a reboot unless saved.

Using
*****

.. code-block:: console

   ssh -i ~/.ssh/id_rsa root@espressobinf0ad4e03858d.local
   zephyr:~$ fwupdate tcp
   $ nc -N espressobinf0ad4e03858d.local 6969 < flash-image.bin
   zephyr:~$ kernel reboot cold

Limitations
***********

There is no hardware RNG driver yet: host key and session keys come from
``CONFIG_TEST_RANDOM_GENERATOR``, which is not cryptographically secure.
Use it on trusted networks only. ``fwupdate tcp`` has no authentication of
its own; it only listens after being started from the shell.
