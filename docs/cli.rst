CLI interface
=============

You can use the CLI tool to test and configure the Allegro hand.

Usage
-----

.. code-block:: console

   allegro_cli [-l] <can_interface>
   allegro_cli [-ipv] [-d new_device_id] <can_interface> [<device_id>]

Options
-------

.. code-block:: console

   -l            List devices
   -i            Print device info
   -p            Print joint positions
   -v            Print joint velocities
   -d id         Set device id

Example
-------

- List devices ::code:`allegro_cli -l can0`

- Device info ::code:`allegro_cli -i can0 0`

- Print joint positions ::code:`allegro_cli -p can0 0`

- Print joint velocities ::code:`allegro_cli -v can0 0`

- Change device ID from 0 to 2 ::code:`allegro_cli -d 2 can0 0`
