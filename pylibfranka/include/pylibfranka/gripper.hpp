// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Expose gripper-related python bindings

#pragma once
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace pylibfranka {

void bind_gripper(py::module& m);

}