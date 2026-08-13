#include "window_soft_surface_mode_banner.hpp"

#if HAS_SOFT_SURFACE_MODE()

#include "align.hpp"
#include "display.hpp"
#include "gui.hpp"

WindowSoftSurfaceModeBanner::WindowSoftSurfaceModeBanner(window_t *parent, Rect16 rect)
    : window_text_t(parent, rect, is_multiline::no, is_closed_on_click_t::no, string_view_utf8::MakeCPUFLASH("SoftSurfaceMode(Tijn)")) {
    SetAlignment(Align_t::Center());
    SetTextColor(COLOR_RED_ALERT);
}

void WindowSoftSurfaceModeBanner::unconditionalDraw() {
    if (!blink_phase) {
        // fully blank - no border, no leftover text - this is the "0.5s off" half of the blink
        display::fill_rect(GetRect(), GetParent() ? GetParent()->GetBackColor() : GetBackColor());
        return;
    }

    window_text_t::unconditionalDraw();
    display::draw_rect(GetRect(), COLOR_RED_ALERT);
}

void WindowSoftSurfaceModeBanner::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    const bool prev_blink_phase = blink_phase;
    blink_phase = (gui::GetTick() / uint32_t(500)) & 0b1;

    if (blink_phase != prev_blink_phase) {
        Invalidate();
    }

    window_text_t::windowEvent(sender, event, param);
}

#endif
