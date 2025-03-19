// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <libfcimodels.h>

#include "model_library_interface.h"

#undef LIBFRANKA_EXPORT
#ifdef LIBFRANKA_WINDOWS
#define LIBFRANKA_EXPORT __declspec(dllexport)
#else
#define LIBFRANKA_EXPORT
#endif

LIBFRANKA_EXPORT ModelLibraryInterface* model_library_interface;

void Ji_J_J1(double b_Ji_J_J1[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J1(b_Ji_J_J1);
  }
}
void Ji_J_J2(const double q[7], double b_Ji_J_J2[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J2(q, b_Ji_J_J2);
  }
}
void Ji_J_J3(const double q[7], double b_Ji_J_J3[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J3(q, b_Ji_J_J3);
  }
}
void Ji_J_J4(const double q[7], double b_Ji_J_J4[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J4(q, b_Ji_J_J4);
  }
}
void Ji_J_J5(const double q[7], double b_Ji_J_J5[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J5(q, b_Ji_J_J5);
  }
}
void Ji_J_J6(const double q[7], double b_Ji_J_J6[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J6(q, b_Ji_J_J6);
  }
}
void Ji_J_J7(const double q[7], double b_Ji_J_J7[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J7(q, b_Ji_J_J7);
  }
}
void Ji_J_J8(const double q[7], double b_Ji_J_J8[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J8(q, b_Ji_J_J8);
  }
}
void Ji_J_J9(const double q[7], const double F_T_EE[16], double b_Ji_J_J9[42]) {
  if (model_library_interface) {
    model_library_interface->Ji_J_J9(q, F_T_EE, b_Ji_J_J9);
  }
}

void O_J_J1(double b_O_J_J1[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J1(b_O_J_J1);
  }
}
void O_J_J2(const double q[7], double b_O_J_J2[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J2(q, b_O_J_J2);
  }
}
void O_J_J3(const double q[7], double b_O_J_J3[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J3(q, b_O_J_J3);
  }
}
void O_J_J4(const double q[7], double b_O_J_J4[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J4(q, b_O_J_J4);
  }
}
void O_J_J5(const double q[7], double b_O_J_J5[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J5(q, b_O_J_J5);
  }
}
void O_J_J6(const double q[7], double b_O_J_J6[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J6(q, b_O_J_J6);
  }
}
void O_J_J7(const double q[7], double b_O_J_J7[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J7(q, b_O_J_J7);
  }
}
void O_J_J8(const double q[7], double b_O_J_J8[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J8(q, b_O_J_J8);
  }
}
void O_J_J9(const double q[7], const double F_T_EE[16], double b_O_J_J9[42]) {
  if (model_library_interface) {
    model_library_interface->O_J_J9(q, F_T_EE, b_O_J_J9);
  }
}

void O_T_J1(const double q[7], double O_T_J1[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J1(q, O_T_J1);
  }
}

void O_T_J2(const double q[7], double O_T_J2[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J2(q, O_T_J2);
  }
}

void O_T_J3(const double q[7], double O_T_J3[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J3(q, O_T_J3);
  }
}

void O_T_J4(const double q[7], double O_T_J4[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J4(q, O_T_J4);
  }
}

void O_T_J5(const double q[7], double O_T_J5[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J5(q, O_T_J5);
  }
}

void O_T_J6(const double q[7], double O_T_J6[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J6(q, O_T_J6);
  }
}

void O_T_J7(const double q[7], double O_T_J7[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J7(q, O_T_J7);
  }
}

void O_T_J8(const double q[7], double O_T_J8[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J8(q, O_T_J8);
  }
}

void O_T_J9(const double q[7], const double F_T_EE[16], double O_T_J9[16]) {
  if (model_library_interface) {
    model_library_interface->O_T_J9(q, F_T_EE, O_T_J9);
  }
}
