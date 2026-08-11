#include "dialog_soft_surface_probing.hpp"

#include <algorithm>
#include <cmath>

#include "display.hpp"
#include "sound.hpp"
#include "sensor_data.hpp"
#include "loadcell.hpp"
#include <gui/event/gui_event.hpp>
#include <gui/event/knob_event.hpp>
#include <guiconfig/GuiDefaults.hpp>
#include <config_store/store_instance.hpp>

namespace {
constexpr uint16_t bar_width = 16;
constexpr uint16_t gauge_width = 40;
constexpr uint16_t arrow_size = 8; // half-height and length of the threshold-arrow triangle
constexpr uint16_t baseline_x_offset = bar_width + 4;

/// Visual scale top = soft_surface_max_probe_force (the existing hard-abort ceiling) - the bar
/// can never exceed the height the printer would itself abort probing at.
float scale_max_g() {
    return std::max(1.f, config_store().soft_surface_max_probe_force.get());
}

uint16_t relative_to_px(float value_g, float scale_max, uint16_t height) {
    const float relative = std::clamp(value_g / scale_max, 0.f, 1.f);
    return static_cast<uint16_t>(relative * height);
}

Rect16 filled_bar_rect(const Rect16 &rect, uint16_t filled_px) {
    // fills from the bottom up
    return Rect16(rect.Left(), rect.Top() + rect.Height() - filled_px, bar_width, filled_px);
}

Rect16 empty_bar_rect(const Rect16 &rect, uint16_t filled_px) {
    return Rect16(rect.Left(), rect.Top(), bar_width, rect.Height() - filled_px);
}

void draw_threshold_arrow(uint16_t baseline_x, uint16_t center_y, Color clr) {
    // Left-pointing filled triangle, apex touching the baseline, base extending right.
    for (uint16_t dx = 0; dx <= arrow_size; ++dx) {
        const uint16_t half_height = dx;
        display::fill_rect(Rect16(baseline_x + dx, center_y - half_height, 1, 2 * half_height + 1), clr);
    }
}
} // namespace

/*****************************************************************************/
// WindowSoftSurfaceGauge

WindowSoftSurfaceGauge::WindowSoftSurfaceGauge(window_t *parent, Rect16 rect, float initial_threshold_g)
    : window_frame_t(parent, rect)
    , threshold_g(initial_threshold_g) {
    SetBackColor(COLOR_BLACK);
}

bool WindowSoftSurfaceGauge::Change(int diff) {
    const auto &config = soft_surface_probe_force_spin_config;
    const float previous = threshold_g;
    threshold_g = std::round(threshold_g / config.step + diff) * config.step;
    threshold_g = config.clamp(threshold_g, static_cast<float>(diff));

    if (threshold_g != previous) {
        // Immediate real-time effect on the *next* probe/homing attempt within this session.
        loadcell.SetLiveProbeForceOverride(threshold_g);
        Invalidate();
        return true;
    }
    return false;
}

void WindowSoftSurfaceGauge::UpdateLiveForce() {
    const float force_g = sensor_data().loadCell.load();
    if (force_g != last_drawn_force_g) {
        last_drawn_force_g = force_g;
        Invalidate();
    }
}

void WindowSoftSurfaceGauge::unconditionalDraw() {
    const Rect16 rect = GetRect();
    const uint16_t baseline_x = rect.Left() + baseline_x_offset;
    const float scale_max = scale_max_g();

    // live force bar ("dikke staaf"), filled from the bottom
    const uint16_t force_px = relative_to_px(std::max(0.f, last_drawn_force_g), scale_max, rect.Height());
    display::fill_rect(empty_bar_rect(rect, force_px), COLOR_DARK_GRAY);
    display::fill_rect(filled_bar_rect(rect, force_px), COLOR_BRAND);

    // short vertical baseline
    display::draw_line(point_ui16(baseline_x, rect.Top()), point_ui16(baseline_x, rect.Top() + rect.Height()), COLOR_WHITE);

    // threshold arrow
    const uint16_t threshold_px = relative_to_px(threshold_g, scale_max, rect.Height());
    const uint16_t threshold_y = rect.Top() + rect.Height() - threshold_px;
    draw_threshold_arrow(baseline_x, std::clamp<uint16_t>(threshold_y, rect.Top() + arrow_size, rect.Top() + rect.Height() - arrow_size), COLOR_RED);
}

/*****************************************************************************/
// DialogSoftSurfaceProbing

namespace {
constexpr uint16_t indentation_label_height = 14;

Rect16 gauge_rect() {
    // Compact vertical strip in the upper-right corner of the screen body, clear of header/
    // footer. Exact placement is a first pass - needs visual tuning against a real display or
    // simulator, not verifiable in this headless environment.
    constexpr uint16_t height = 180;
    constexpr uint16_t margin = 10;
    return Rect16(
        GuiDefaults::ScreenWidth - gauge_width - margin,
        GuiDefaults::RectScreenBody.Top() + margin + indentation_label_height,
        gauge_width,
        height);
}

Rect16 indentation_label_rect(const Rect16 &gauge) {
    // Small readout directly above the gauge bar showing the current max-indentation tolerance
    // (mm) - the flatness/consistency limit that can reject a probe point (probe.cpp). Requested
    // so the "how strict is the flatness check" threshold is visible during probing, same as the
    // force threshold already is via the bar+arrow below it.
    return Rect16(gauge.Left(), gauge.Top() - indentation_label_height, gauge.Width(), indentation_label_height);
}
} // namespace

DialogSoftSurfaceProbing::DialogSoftSurfaceProbing(fsm::BaseData /*data*/)
    : IDialogMarlin(gauge_rect().Union(indentation_label_rect(gauge_rect())))
    , gauge(this, gauge_rect(), config_store().soft_surface_probe_force.get())
    , max_indentation_label(this, indentation_label_rect(gauge_rect()), config_store().soft_surface_max_indentation.get(), "%.2f", GuiDefaults::FontMenuSpecial) {
    max_indentation_label.SetTextColor(COLOR_ORANGE);
    // Active from the moment the dialog opens, not only after the first knob turn, so the very
    // first probe/homing attempt in this session already reads the live channel.
    loadcell.SetLiveProbeForceOverride(gauge.GetValue());
}

DialogSoftSurfaceProbing::~DialogSoftSurfaceProbing() {
    // "Save on leaving the probing session".
    config_store().soft_surface_probe_force.set(gauge.GetValue());
    loadcell.SetLiveProbeForceOverride(std::nullopt);
}

void DialogSoftSurfaceProbing::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {

    case GUI_event_t::KNOB: {
        auto &ctx = *static_cast<GuiEventContext *>(param);
        auto &ev = ctx.event.value<gui_event::KnobEvent>();
        const bool changed = gauge.Change(ev.diff);
        sound::play(changed ? SoundType::encoder_move : SoundType::blind_alert);
        ctx.accept();
        break;
    }

    case GUI_event_t::LOOP:
        gauge.UpdateLiveForce();
        break;

    default:
        break;
    }

    IDialogMarlin::windowEvent(sender, event, param);
}
