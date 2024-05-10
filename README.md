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

After setting up CAN interface, restart networking

```
sudo systemctl restart systemd-networkd.service
```

### CLI interface

You can use the CLI tool to test and configure the Allegro hand.
Usage:
```
allegro_cli [-l] <can_interface>
allegro_cli [-ipv] [-d new_device_id] <can_interface> [<device_id>]
```

Options:
```
  -l            List devices
  -i            Print device info
  -p            Print joint positions
  -v            Print joint velocities
  -d id         Set device id
```

Example
 - List devices: `allegro_cli -l can0`
 - Device info: `allegro_cli -i can0 0`
 - Print joint positions: `allegro_cli -p can0 0`
 - Print joint velocities: `allegro_cli -v can0 0`
 - Set change device ID from 0 to 2: `allegro_cli -d 2 can0 0`