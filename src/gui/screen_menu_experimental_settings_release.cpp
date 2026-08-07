/**
 * @file screen_menu_experimental_settings_release.cpp
 */

#include "screen_menu_experimental_settings.hpp"
#include "WindowMenuSpin.hpp"
#include "ScreenHandler.hpp"
#include "window_msgbox.hpp"
#include <common/sys.hpp>
#include "string.h" // memcmp
#include "MItem_experimental_tools.hpp"

void ScreenMenuExperimentalSettings::clicked_return() {
    ExperimentalSettingsValues current(*this); // ctor will handle load of values
    // unchanged
    if (current == initial) {
        Screens::Access()->Close();
        return;
    }

    switch (MsgBoxQuestion(_(save_and_reboot), Responses_YesNoCancel)) {
    case Response::Yes:
        Item<MI_Z_AXIS_LEN>().Store();
        Item<MI_STEPS_PER_UNIT_E>().Store();
        Item<MI_DIRECTION_E>().Store();
#if HAS_LOADCELL()
        Item<MI_LOADCELL_SCALE>().Store();
#endif // HAS_LOADCELL()

#if HAS_SOFT_SURFACE_MODE()
        Item<MI_SOFT_SURFACE_PROBE_FORCE>().Store();
        Item<MI_SOFT_SURFACE_MAX_PROBE_FORCE>().Store();
        Item<MI_SOFT_SURFACE_PROBE_SPEED>().Store();
        Item<MI_SOFT_SURFACE_PROBE_SAMPLES>().Store();
        Item<MI_SOFT_SURFACE_FILTER_STRENGTH>().Store();
        Item<MI_SOFT_SURFACE_COMPRESSION_COMPENSATION>().Store();
        Item<MI_SOFT_SURFACE_MAX_INDENTATION>().Store();
#endif // HAS_SOFT_SURFACE_MODE()

        sys_reset();
    case Response::No:
        Screens::Access()->Close();
        return;
    default:
        return; // do nothing
    }
}

ScreenMenuExperimentalSettings::ScreenMenuExperimentalSettings()
    : ScreenMenuExperimentalSettings__(_(label))
    , initial(*this) {}

void ScreenMenuExperimentalSettings::windowEvent(window_t *sender, GUI_event_t ev, void *param) {
    if (ev != GUI_event_t::CHILD_CLICK) {
        ScreenMenu::windowEvent(sender, ev, param);
        return;
    }

    switch (ClickCommand(intptr_t(param))) {
    case ClickCommand::Return:
        clicked_return();
        break;
    case ClickCommand::Reset_Z:
        Item<MI_Z_AXIS_LEN>().SetVal(DEFAULT_Z_MAX_POS);
        Invalidate();
        break;
    case ClickCommand::Reset_steps:
        Item<MI_STEPS_PER_UNIT_E>().SetVal(std::abs(config_store().axis_steps_per_unit_e0.default_val));
        Invalidate();
        break;
    case ClickCommand::Reset_directions:
        Item<MI_DIRECTION_E>().set_current_item(0);
        Invalidate();
        break;
    default:
        break;
    }
}

bool ExperimentalSettingsValues::operator==(const ExperimentalSettingsValues &other) const {
    return memcmp(this, &other, sizeof(ExperimentalSettingsValues)) == 0;
}
bool ExperimentalSettingsValues::operator!=(const ExperimentalSettingsValues &other) const {
    return !(*this == other);
}

ExperimentalSettingsValues::ExperimentalSettingsValues(ScreenMenuExperimentalSettings__ &parent)
    : z_len(static_cast<int32_t>(parent.Item<MI_Z_AXIS_LEN>().GetVal()))
    , steps_per_unit_e(static_cast<int32_t>(parent.Item<MI_STEPS_PER_UNIT_E>().GetVal()) * ((parent.Item<MI_DIRECTION_E>().get_index() == 1) ? -1 : 1))
#if HAS_SOFT_SURFACE_MODE()
    , soft_surface_probe_force(parent.Item<MI_SOFT_SURFACE_PROBE_FORCE>().value())
    , soft_surface_max_probe_force(parent.Item<MI_SOFT_SURFACE_MAX_PROBE_FORCE>().value())
    , soft_surface_probe_speed(parent.Item<MI_SOFT_SURFACE_PROBE_SPEED>().value())
    , soft_surface_probe_samples(static_cast<int32_t>(parent.Item<MI_SOFT_SURFACE_PROBE_SAMPLES>().value()))
    , soft_surface_filter_strength(parent.Item<MI_SOFT_SURFACE_FILTER_STRENGTH>().value())
    , soft_surface_compression_compensation(parent.Item<MI_SOFT_SURFACE_COMPRESSION_COMPENSATION>().value())
    , soft_surface_max_indentation(parent.Item<MI_SOFT_SURFACE_MAX_INDENTATION>().value())
#endif
{}
