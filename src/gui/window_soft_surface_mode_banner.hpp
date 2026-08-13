/**
 * @file window_soft_surface_mode_banner.hpp
 * @brief Always-visible blinking warning banner shown in the header while Soft Surface Mode is enabled.
 */
#pragma once

#include "window_text.hpp"
#include <option/has_soft_surface_mode.h>

#if HAS_SOFT_SURFACE_MODE()

/// Blinking "SoftSurfaceMode(Tijn)" banner (0.5s on / 0.5s off), drawn with a border so it's
/// obvious at a glance - to anyone using the printer - that the firmware is not running standard
/// behavior. The owner (window_header_t) controls whether this widget is shown at all, based on
/// config_store().soft_surface_mode_enabled; this class only handles the blink animation itself.
class WindowSoftSurfaceModeBanner : public window_text_t {
    bool blink_phase : 1 = false;

public:
    WindowSoftSurfaceModeBanner(window_t *parent, Rect16 rect);

protected:
    virtual void unconditionalDraw() override;
    virtual void windowEvent(window_t *sender, GUI_event_t event, void *param) override;
};

#endif
