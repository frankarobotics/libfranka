.. _realtime_kernel_setup:

Setting up the Real-Time Kernel
-------------------------------

Kernel Compatibility
~~~~~~~~~~~~~~~~~~~~

Different kernel versions are compatible with specific Ubuntu releases. The table below provides an overview of the recommended kernel versions for each supported Ubuntu distribution.

+----------------+-------------------------+
| Kernel Version | Ubuntu Version          |
+================+=========================+
| Pro Kernel     | 22.04 (Jammy Jellyfish) |
+----------------+-------------------------+
| 5.9.1          | 20.04 (Focal Fossa)     |
+----------------+-------------------------+
| 5.4.19         | 18.04 (Bionic Beaver)   |
+----------------+-------------------------+
| 4.14.12        | 16.04 (Xenial Xerus)    |
+----------------+-------------------------+

To control your robot using ``libfranka``, the controller program on the
workstation PC must run with `real-time priority` under a ``PREEMPT_RT``
kernel. This section describes how to patch a kernel and create an installation package.

.. note::

    NVIDIA drivers are not officially supported on ``PREEMPT_RT`` kernels, but installation might work by passing the environment variable ``IGNORE_PREEMPT_RT_PRESENCE=1`` to the installation command.

First, install the necessary dependencies::

    sudo apt-get install build-essential bc curl debhelper dpkg-dev devscripts \
    fakeroot libssl-dev libelf-dev bison flex cpio kmod rsync libncurses-dev

Then decide which kernel version to use. Check your current version with ``uname -r``.
Real-time patches are only available for select versions: `RT Kernel Patches <https://www.kernel.org/pub/linux/kernel/projects/rt/>`_.
Choose the version closest to your current kernel. Use ``curl`` to download the sources.

.. note::
   Ubuntu 16.04 (tested with kernel 4.14.12):

   .. code-block:: bash

      curl -LO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign

.. note::
   Ubuntu 18.04 (tested with kernel 5.4.19):

   .. code-block:: bash

      curl -LO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.sign
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.sign

.. note::
   Ubuntu 20.04 (tested with kernel 5.9.1):

   .. code-block:: bash

      curl -LO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign

.. note::
   Ubuntu 22.04: We recommend using the `Ubuntu Pro real-time kernel <https://ubuntu.com/real-time>`_.
   After enabling it, you can skip directly to :ref:`installation-real-time`.
   If you prefer not to use Ubuntu Pro, you can patch manually (tested with kernel 6.8.0):

   .. code-block:: bash

      curl -LO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.8.tar.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.8.tar.sign
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patch-6.8-rt8.patch.xz
      curl -LO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patch-6.8-rt8.patch.sign

Decompress the files::

    xz -d *.xz

Verifying File Integrity
~~~~~~~~~~~~~~~~~~~~~~~~
.. note::
    Optional but recommended!

Use the ``.sign`` files to verify downloads. Adapted from the
`Linux Kernel Archive <https://www.kernel.org/signature.html>`_.

Verify archives::

    gpg2 --verify linux-*.tar.sign
    gpg2 --verify patch-*.patch.sign

If you get a "No public key" warning, download the key (example for kernel 4.14.12):

    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E
    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 2872E4CC

Then verify again. A correct output looks like::

    gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>"

For details, see the `Linux Kernel Archive <https://www.kernel.org/signature.html>`_.

Compiling the Kernel
~~~~~~~~~~~~~~~~~~~~

Extract the source and apply the patch::

    tar xf linux-*.tar
    cd linux-*/
    patch -p1 < ../patch-*.patch

Copy your current kernel config::

    cp -v /boot/config-$(uname -r) .config

Disable debug info::

    scripts/config --disable DEBUG_INFO
    scripts/config --disable DEBUG_INFO_DWARF_TOOLCHAIN_DEFAULT
    scripts/config --disable DEBUG_KERNEL

Disable system-wide trusted keys::

    scripts/config --disable SYSTEM_TRUSTED_KEYS
    scripts/config --disable SYSTEM_REVOCATION_LIST

Enable the Fully Preemptible Kernel::

    scripts/config --disable PREEMPT_NONE
    scripts/config --disable PREEMPT_VOLUNTARY
    scripts/config --disable PREEMPT
    scripts/config --enable PREEMPT_RT

Configure the build::

    make olddefconfig

Compile the kernel (use multithreading based on CPU cores)::

    make -j$(nproc) deb-pkg

Install the generated packages::

    sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg -i ../linux-headers-*.deb ../linux-image-*.deb

Verifying the New Kernel
~~~~~~~~~~~~~~~~~~~~~~~~

Reboot and select the new kernel in GRUB. Check with ``uname -a`` — it should include ``PREEMPT RT``.
Also, ``/sys/kernel/realtime`` should exist and contain ``1``.

.. note::
    If boot fails, see :ref:`troubleshooting_realtime_kernel`.

.. _installation-real-time:

Allowing Real-Time Permissions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After the kernel is running, create a `realtime` group and add the user controlling the robot::

    sudo addgroup realtime
    sudo usermod -a -G realtime $(whoami)

Add limits in ``/etc/security/limits.conf``::

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

Log out and back in to apply the limits.
