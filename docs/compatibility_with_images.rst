.. _libfranka_compatibility:
Robot System Version Compatibility
-----------------------------------

Several components in the Franka ecosystem evolve independently, and versions may not always be compatible. It is recommended to use the most recent versions of all components.

The symbol ``>=`` indicates that newer Robot System Versions have **not** been explicitly tested with the corresponding ``libfranka`` release. Compatibility *may* work but is **not guaranteed** (e.g., ``libfranka 0.2.0`` may not be compatible with Robot System version ``4.0.0``).

The table below shows tested compatibility between libfranka versions, Robot System versions, and Robot/Gripper servers. All versions use ``>=`` and ``<`` where applicable to indicate tested ranges.

.. list-table::
   :header-rows: 1
   :widths: 25 25 20
   :stub-columns: 0

   * - libfranka Version
     - Robot System Version
     - Robot/Gripper Server

   * - >= 0.18.0
     - >= 5.9.0
     - 10 / 3

   * - >= 0.15.0 < 0.18.0
     - >= 5.7.2 < 5.9.0
     - 9 / 3

   * - >= 0.14.1 < 0.15.0
     - >= 5.7.0 < 5.7.2
     - 8 / 3

   * - >= 0.13.3 < 0.14.1
     - >= 5.5.0 < 5.7.0
     - 7 / 3

   * - >= 0.10.0 < 0.13.3
     - >= 5.2.0 < 5.5.0
     - 6 / 3

   * - >= 0.9.1 < 0.10.0
     - >= 4.2.1 < 5.2.0
     - 5 / 3

   * - >= 0.8.0 < 0.9.1
     - >= 4.0.0 < 4.2.1
     - 4 / 3

   * - >= 0.7.1 < 0.8.0
     - >= 3.0.0 < 4.0.0
     - 3 / 3

   * - >= 0.5.0 < 0.7.1
     - >= 1.3.0 < 3.0.0
     - 3 / 2

   * - >= 0.3.0 < 0.5.0
     - >= 1.2.0 < 1.3.0
     - 2 / 2

   * - >= 0.2.0 < 0.3.0
     - >= 1.1.0 < 1.2.0
     - 2 / 1
