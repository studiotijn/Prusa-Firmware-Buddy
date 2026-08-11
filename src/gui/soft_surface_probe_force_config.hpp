// Bookmark3D Soft Surface Mode: shared bounds/step for editing soft_surface_probe_force, so the
// Experimental Settings menu item (MI_SOFT_SURFACE_PROBE_FORCE, MItem_experimental_tools.cpp)
// and the live probing-force gauge overlay (dialog_soft_surface_probing.cpp) can't drift apart.
#pragma once

#include <numeric_input_config.hpp>

inline constexpr NumericInputConfig soft_surface_probe_force_spin_config {
    .min_value = 1,
    .max_value = 500,
    .max_decimal_places = 1,
};
