Installation
============

Using CMake
-----------

1. Install CMake and a C++ compiler that supports C++17.
2. Create a build directory and navigate into it:

   .. code-block:: bash

      mkdir build
      cd build

3. Run CMake to configure the project:

   .. code-block:: bash

      cmake ..

4. Build the project:

   .. code-block:: bash

      cmake --build .

5. Optionally, install the library and tools:

   .. code-block:: bash

      sudo cmake --install .

Using Colcon
------------

The repository is a valid Colcon package. To build it put the package in your `Colcon workspace`_ and `build the workspace`_.

.. _Colcon workspace: https://colcon.readthedocs.io/en/released/user/what-is-a-workspace.html
.. _build the workspace: https://colcon.readthedocs.io/en/released/user/quick-start.html

CAN interface setup
-------------------

For CAN interfaces supported by the Linux kernel (e.g. `Peak CAN interfaces`_), configure the interface using systemd.
Create `/etc/systemd/network/80-can.network`, with the following configuration for can0:

   .. code-block:: ini

      [Match]
      Name=can0
      [CAN]
      BitRate=1000K
      RestartSec=100ms

After setting up CAN interface, restart networking:

   .. code-block:: bash

      sudo systemctl restart systemd-networkd.service

.. _Peak CAN interfaces: https://www.peak-system.com/PCAN-USB.199.0.html?&L=1

.. note::

      If you are running the API or derived driver in a Docker container,
      you have to perform the bove steps in the host system.
      You also need to run the container with `--network host` option to allow
      the container to access the host CAN interface.
