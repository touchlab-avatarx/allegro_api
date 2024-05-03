# Allegro c++ API

### CAN interface setup
For CAN interfaces supported by the Linux kernel (e.g. Peak CAN interfaces), configure the interface using systemd.
Create `/etc/systemd/network/80-can.network`, with the following configuration for can0:
```
[Match]
Name=can0
[CAN]
BitRate=1000K
RestartSec=100ms
```
