// Include relevant boost libraries required by GTSAM
{include_boost}

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>

#include "gtsam/base/utilities.h"

// These are the included headers listed in `gtsam_quadrics.i`
{includes}

// Export classes for serialization
#include <boost/serialization/export.hpp>
{boost_class_export}

// Holder type for pybind11
{holder_type}

// Preamble for STL classes
// TODO(fan): make this automatic
// #include "python/gtsam/preamble/{module_name}.h"

using namespace std;

namespace py = pybind11;

{submodules}

{module_def} {{
    m_.doc() = "pybind11 wrapper of {module_name}";

    // At the moment we have to import the gtsam Python module before importing
    // gtsam_quadrics, as there are errors about finding gtsam classes. The
    // PyBind solution is something like below, but currently the
    // auto-scripting in gtwrap gets in our way:
    //   https://pybind11.readthedocs.io/en/stable/advanced/misc.html#partitioning-code-over-multiple-extension-modules
    // py::module::import("gtsam").attr("NoiseModelFactor");

{submodules_init}

{wrapped_namespace}

// Specializations for STL classes
// TODO(fan): make this automatic
// #include "python/gtsam/specializations/{module_name}.h"

}}

