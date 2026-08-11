// Bookmark3D Soft Surface Mode: live loadcell-force gauge shown during a soft-surface G28/G29
// probing session (see docs/design.md). Modeled on liveadjust_z.hpp/.cpp (WindowScale +
// WindowLiveAdjustZ): a short vertical scale line, a live-updating force bar, a threshold
// arrow adjustable via the knob, and persist-on-close semantics.
#pragma once

#include "IDialogMarlin.hpp"
#include "soft_surface_probe_force_config.hpp"

/// Short vertical scale + live force bar ("dikke staaf") + threshold arrow.
class WindowSoftSurfaceGauge : public window_frame_t {
public:
    WindowSoftSurfaceGauge(window_t *parent, Rect16 rect, float initial_threshold_g);

    float GetValue() const { return threshold_g; }

    /// Adjust the threshold by a knob diff (positive = clockwise = more force, per
    /// gui_event::KnobEvent's documented convention). Immediately pushes the new value to
    /// Loadcell's live override (loadcell.hpp) so it takes effect on the next probe/homing
    /// attempt within this session. @return true if the value actually changed.
    bool Change(int diff);

    /// Polls the live loadcell force (sensor_data().loadCell) and redraws the bar if it moved.
    /// Call on every GUI_event_t::LOOP tick.
    void UpdateLiveForce();

protected:
    virtual void unconditionalDraw() override;

private:
    float threshold_g;
    float last_drawn_force_g = -1.f; // sentinel: guarantees the first LOOP tick always redraws
};

/// Host dialog, bound to ClientFSM::SoftSurfaceProbing / PhaseSoftSurfaceProbing via
/// DialogHandler's FSMDisplayConfig table (see DialogHandler.cpp). Opened/closed automatically
/// by marlin_server::FSM_Holder on the core side for the duration of one probing session
/// (see ubl_G29.cpp's probe_major_points() and G28.cpp's home_z_safely()).
class DialogSoftSurfaceProbing : public IDialogMarlin {
public:
    DialogSoftSurfaceProbing(fsm::BaseData data);

    /// Persists the live-adjusted threshold to config_store and releases the live override -
    /// "save on leaving the probing session".
    ~DialogSoftSurfaceProbing();

protected:
    void windowEvent(window_t *sender, GUI_event_t event, void *param) override;

private:
    WindowSoftSurfaceGauge gauge;
};
