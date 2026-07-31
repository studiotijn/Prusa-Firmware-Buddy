/// @file
#pragma once

#include <cstdint>

// Bookmark3D (experimental, see docs/design.md): selects whether the bed heater is used.
// !!! DO NOT REORDER, DO NOT CHANGE - this is used in config store
enum class BedType : uint8_t {
    standard,
    soft_surface, // Bookmark3D Soft Surface Mode: no heatbed, M140/M190 become no-ops
};
