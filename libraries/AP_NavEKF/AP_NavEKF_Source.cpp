/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_NavEKF_Source.h"
#include <AP_Math/AP_Math.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Nav_Common.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NavEKF_Source::var_info[] = {

    // @Param: 1_POSXY
    // @DisplayName: Position Horizontal Source (Primary)
    // @Description: Position Horizontal Source (Primary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("1_POSXY", 1, AP_NavEKF_Source, _source_set[0].posxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: 1_VELXY
    // @DisplayName: Velocity Horizontal Source
    // @Description: Velocity Horizontal Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("1_VELXY", 2, AP_NavEKF_Source, _source_set[0].velxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: 1_POSZ
    // @DisplayName: Position Vertical Source
    // @Description: Position Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("1_POSZ", 3, AP_NavEKF_Source, _source_set[0].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: 1_VELZ
    // @DisplayName: Velocity Vertical Source
    // @Description: Velocity Vertical Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("1_VELZ", 4, AP_NavEKF_Source, _source_set[0].velz, (int8_t)AP_NavEKF_Source::SourceZ::GPS),

    // @Param: 1_YAW
    // @DisplayName: Yaw Source
    // @Description: Yaw Source
    // @Values: 0:None, 1:Compass, 2:GPS, 3:GPS with Compass Fallback, 6:ExternalNav, 8:GSF
    // @User: Advanced
    AP_GROUPINFO("1_YAW", 5, AP_NavEKF_Source, _source_set[0].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),

#if AP_NAKEKF_SOURCE_SET_MAX >= 2
    // @Param: 2_POSXY
    // @DisplayName: Position Horizontal Source (Secondary)
    // @Description: Position Horizontal Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("2_POSXY", 6, AP_NavEKF_Source, _source_set[1].posxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: 2_VELXY
    // @DisplayName: Velocity Horizontal Source (Secondary)
    // @Description: Velocity Horizontal Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("2_VELXY", 7, AP_NavEKF_Source, _source_set[1].velxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: 2_POSZ
    // @DisplayName: Position Vertical Source (Secondary)
    // @Description: Position Vertical Source (Secondary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("2_POSZ", 8, AP_NavEKF_Source, _source_set[1].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: 2_VELZ
    // @DisplayName: Velocity Vertical Source (Secondary)
    // @Description: Velocity Vertical Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("2_VELZ", 9, AP_NavEKF_Source, _source_set[1].velz, (int8_t)AP_NavEKF_Source::SourceZ::NONE),

    // @Param: 2_YAW
    // @DisplayName: Yaw Source (Secondary)
    // @Description: Yaw Source (Secondary)
    // @Values: 0:None, 1:Compass, 2:GPS, 3:GPS with Compass Fallback, 6:ExternalNav, 8:GSF
    // @User: Advanced
    AP_GROUPINFO("2_YAW", 10, AP_NavEKF_Source, _source_set[1].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::NONE),
#endif

#if AP_NAKEKF_SOURCE_SET_MAX >= 3
    // @Param: 3_POSXY
    // @DisplayName: Position Horizontal Source (Tertiary)
    // @Description: Position Horizontal Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("3_POSXY", 11, AP_NavEKF_Source, _source_set[2].posxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: 3_VELXY
    // @DisplayName: Velocity Horizontal Source (Tertiary)
    // @Description: Velocity Horizontal Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("3_VELXY", 12, AP_NavEKF_Source, _source_set[2].velxy, (int8_t)AP_NavEKF_Source::SourceXY::NONE),

    // @Param: 3_POSZ
    // @DisplayName: Position Vertical Source (Tertiary)
    // @Description: Position Vertical Source (Tertiary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("3_POSZ", 13, AP_NavEKF_Source, _source_set[2].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: 3_VELZ
    // @DisplayName: Velocity Vertical Source (Tertiary)
    // @Description: Velocity Vertical Source (Tertiary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("3_VELZ", 14, AP_NavEKF_Source, _source_set[2].velz, (int8_t)AP_NavEKF_Source::SourceZ::NONE),

    // @Param: 3_YAW
    // @DisplayName: Yaw Source (Tertiary)
    // @Description: Yaw Source (Tertiary)
    // @Values: 0:None, 1:Compass, 2:GPS, 3:GPS with Compass Fallback, 6:ExternalNav, 8:GSF
    // @User: Advanced
    AP_GROUPINFO("3_YAW", 15, AP_NavEKF_Source, _source_set[2].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::NONE),
#endif

    // @Param: _OPTIONS
    // @DisplayName: EKF Source Options
    // @Description: EKF Source Options. Bit 0: Fuse all velocity sources present in EK3_SRCx_VEL_. Bit 1: Align external navigation position when using optical flow. Bit 3: Use SRC per core. By default, EKF source selection is controlled via the EK3_SRC parameters, allowing only one source to be active at a time across all cores (switchable via MAVLink, Lua, or RC). Enabling this bit maps EKF core 1 to SRC1, core 2 to SRC2, etc., allowing each core to run independently with a dedicated source.
    // @Bitmask: 0:FuseAllVelocities, 1:AlignExtNavPosWhenUsingOptFlow, 3: UsePerCoreEKFSources
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 16, AP_NavEKF_Source, _options, 0),

    AP_GROUPEND
};

AP_NavEKF_Source::AP_NavEKF_Source()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
void AP_NavEKF_Source::setPosVelYawSourceSet(AP_NavEKF_Source::SourceSetSelection source_set_idx)
{
    // sanity check source idx
    if ((uint8_t)source_set_idx < AP_NAKEKF_SOURCE_SET_MAX) {
        active_source_set = (uint8_t)source_set_idx;
#if HAL_LOGGING_ENABLED
        static const LogEvent evt[AP_NAKEKF_SOURCE_SET_MAX] {
            LogEvent::EK3_SOURCES_SET_TO_PRIMARY,
            LogEvent::EK3_SOURCES_SET_TO_SECONDARY,
            LogEvent::EK3_SOURCES_SET_TO_TERTIARY,
        };
        AP::logger().Write_Event(evt[active_source_set]);
#endif
    }
}

// true/false of whether velocity source should be used
bool AP_NavEKF_Source::useVelXYSource(SourceXY velxy_source, uint8_t core_index) const
{
    if (velxy_source == _source_set[getActiveSourceSet(core_index)].velxy) {
        return true;
    }

    // check for fuse all velocities
    if (option_is_set(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (_source_set[i].velxy == velxy_source) {
                return true;
            }
        }
    }

    // if we got this far source should not be used
    return false;
}

bool AP_NavEKF_Source::useVelZSource(SourceZ velz_source, uint8_t core_index) const
{
    if (velz_source == _source_set[getActiveSourceSet(core_index)].velz) {
        return true;
    }

    // check for fuse all velocities
    if (option_is_set(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (_source_set[i].velz == velz_source) {
                return true;
            }
        }
    }

    // if we got this far source should not be used
    return false;
}

// true if a velocity source is configured
bool AP_NavEKF_Source::haveVelZSource(uint8_t core_index) const
{
    if (_source_set[getActiveSourceSet(core_index)].velz != SourceZ::NONE) {
        return true;
    }

    // check for fuse all velocities
    if (option_is_set(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (_source_set[i].velz != SourceZ::NONE) {
                return true;
            }
        }
    }

    // if we got this far no velocity z source has been configured
    return false;
}

// get yaw source
AP_NavEKF_Source::SourceYaw AP_NavEKF_Source::getYawSource(uint8_t core_index) const
{
    // check for special case of disabled compasses
    if ((_source_set[getActiveSourceSet(core_index)].yaw == SourceYaw::COMPASS) && (AP::dal().compass().get_num_enabled() == 0)) {
        return SourceYaw::NONE;
    }

    return _source_set[getActiveSourceSet(core_index)].yaw;
}

// get pos Z source
AP_NavEKF_Source::SourceZ AP_NavEKF_Source::getPosZSource(uint8_t core_index) const
{
#ifdef HAL_BARO_ALLOW_INIT_NO_BARO
    // check for special case of missing baro
    if ((_source_set[getActiveSourceSet(core_index)].posz == SourceZ::BARO) && (AP::dal().baro().num_instances() == 0)) {
        return SourceZ::NONE;
    }
#endif
    return _source_set[getActiveSourceSet(core_index)].posz;
}

// align position of inactive sources to ahrs
void AP_NavEKF_Source::align_inactive_sources()
{
#if HAL_VISUALODOM_ENABLED
    auto *visual_odom = AP::dal().visualodom();
    if (!visual_odom || !visual_odom->enabled()) {
        return;
    }

    bool allow_align_posxy = true;
    bool allow_align_posz = true;

    // Check active XY sources across all EKF cores
    for (uint8_t i = 0; i < MAX_EKF_CORES; i++) {
        const SourceXY pos_xy_source = getPosXYSource(i);
        const SourceXY vel_xy_source = getVelXYSource(i);

        if (pos_xy_source == SourceXY::EXTNAV) {
            // ExtNav is actively being used, do not align XY
            allow_align_posxy = false;
            break;
        }

        const bool valid_xy =
            (pos_xy_source == SourceXY::GPS) ||
            (pos_xy_source == SourceXY::BEACON) ||
            ((vel_xy_source == SourceXY::OPTFLOW) &&
             option_is_set(SourceOptions::ALIGN_EXTNAV_POS_WHEN_USING_OPTFLOW));

        if (!valid_xy) {
            allow_align_posxy = false;
            break;
        }
    }

    // Check active Z sources across all EKF cores
    for (uint8_t i = 0; i < MAX_EKF_CORES; i++) {
        const SourceZ pos_z_source = getPosZSource(i);

        if (pos_z_source == SourceZ::EXTNAV) {
            // ExtNav is actively being used, do not align Z
            allow_align_posz = false;
            break;
        }

        const bool valid_z =
            (pos_z_source == SourceZ::BARO) ||
            (pos_z_source == SourceZ::RANGEFINDER) ||
            (pos_z_source == SourceZ::GPS) ||
            (pos_z_source == SourceZ::BEACON);

        if (!valid_z) {
            allow_align_posz = false;
            break;
        }
    }

    // Check if ExtNav is available in any source set
    bool extnav_available_xy = false;
    bool extnav_available_z = false;

    for (uint8_t i = 0; i < AP_NAKEKF_SOURCE_SET_MAX; i++) {
        if (_source_set[i].posxy == SourceXY::EXTNAV) {
            extnav_available_xy = true;
        }
        if (_source_set[i].posz == SourceZ::EXTNAV) {
            extnav_available_z = true;
        }
    }

    // Align only if allowed and ExtNav source available
    visual_odom->align_position_to_ahrs(
        allow_align_posxy && extnav_available_xy,
        allow_align_posz && extnav_available_z
    );

#endif
}

// sensor specific helper functions
bool AP_NavEKF_Source::usingGPS(uint8_t core_index) const
{
    return getPosXYSource(core_index) == SourceXY::GPS ||
           getPosZSource(core_index) == SourceZ::GPS ||
           getVelXYSource(core_index) == SourceXY::GPS ||
           getVelZSource(core_index) == SourceZ::GPS ||
           getYawSource(core_index) == SourceYaw::GSF;
}

// true if some parameters have been configured (used during parameter conversion)
bool AP_NavEKF_Source::configured()
{
    if (_configured) {
        return true;
    }

    // first source parameter is used to determine if configured or not
    _configured = _source_set[0].posxy.configured();

    return _configured;
}

// mark parameters as configured (used to ensure parameter conversion is only done once)
void AP_NavEKF_Source::mark_configured()
{
    // save first parameter's current value to mark as configured
    return _source_set[0].posxy.save(true);
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
// requires_position should be true if vertical or horizontal position configuration should be checked
bool AP_NavEKF_Source::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    auto &dal = AP::dal();
    bool baro_required = false;
    bool beacon_required = false;
    bool compass_required = false;
    bool gps_required = false;
    bool rangefinder_required = false;
    bool visualodom_required = false;
    bool optflow_required = false;
    bool wheelencoder_required = false;

    // check source params are valid
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {

        if (requires_position) {
            // check posxy
            switch ((SourceXY)_source_set[i].posxy.get()) {
            case SourceXY::NONE:
                break;
            case SourceXY::GPS:
                gps_required = true;
                break;
            case SourceXY::BEACON:
                beacon_required = true;
                break;
            case SourceXY::EXTNAV:
                visualodom_required = true;
                break;
            case SourceXY::OPTFLOW:
            case SourceXY::WHEEL_ENCODER:
            default:
                // invalid posxy value
                hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%d_POSXY", (int)i+1);
                return false;
            }

            // check velxy
            switch ((SourceXY)_source_set[i].velxy.get()) {
            case SourceXY::NONE:
                break;
            case SourceXY::GPS:
                gps_required = true;
                break;
            case SourceXY::OPTFLOW:
                optflow_required = true;
                break;
            case SourceXY::EXTNAV:
                visualodom_required = true;
                break;
            case SourceXY::WHEEL_ENCODER:
                wheelencoder_required = true;
                break;
            case SourceXY::BEACON:
            default:
                // invalid velxy value
                hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%d_VELXY", (int)i+1);
                return false;
            }

            // check posz
            switch ((SourceZ)_source_set[i].posz.get()) {
            case SourceZ::BARO:
                baro_required = true;
                break;
            case SourceZ::RANGEFINDER:
                rangefinder_required = true;
                break;
            case SourceZ::GPS:
                gps_required = true;
                break;
            case SourceZ::BEACON:
                beacon_required = true;
                break;
            case SourceZ::EXTNAV:
                visualodom_required = true;
                break;
            case SourceZ::NONE:
                break;
            default:
                // invalid posz value
                hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%d_POSZ", (int)i+1);
                return false;
            }

            // check velz
            switch ((SourceZ)_source_set[i].velz.get()) {
            case SourceZ::NONE:
                break;
            case SourceZ::GPS:
                gps_required = true;
                break;
            case SourceZ::EXTNAV:
                visualodom_required = true;
                break;
            case SourceZ::BARO:
            case SourceZ::RANGEFINDER:
            case SourceZ::BEACON:
            default:
                // invalid velz value
                hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%d_VELZ", (int)i+1);
                return false;
            }
        }

        // check yaw
        switch ((SourceYaw)_source_set[i].yaw.get()) {
        case SourceYaw::NONE:
        case SourceYaw::GPS:
            // valid yaw value
            break;
        case SourceYaw::COMPASS:
            // skip compass check for easier user setup of compass-less operation
            break;
        case SourceYaw::GPS_COMPASS_FALLBACK:
            compass_required = true;
            break;
        case SourceYaw::EXTNAV:
            visualodom_required = true;
            break;
        case SourceYaw::GSF:
            gps_required = true;
            break;
        default:
            // invalid yaw value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%d_YAW", (int)i+1);
            return false;
        }
    }

    // check all required sensors are available
    const char* ekf_requires_msg = "EK3 sources require %s";
    if (baro_required && (dal.baro().num_instances() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Baro");
        return false;
    }

    if (beacon_required) {
#if AP_BEACON_ENABLED
        const bool beacon_available = (dal.beacon() != nullptr && dal.beacon()->enabled());
#else
        const bool beacon_available = false;
#endif
        if (!beacon_available) {
            hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Beacon");
            return false;
        }
    }

    if (compass_required && (dal.compass().get_num_enabled() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Compass");
        return false;
    }

    if (gps_required && (dal.gps().num_sensors() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "GPS");
        return false;
    }

    if (optflow_required && !dal.opticalflow_enabled()) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "OpticalFlow");
        return false;
    }

    if (rangefinder_required) {
#if AP_RANGEFINDER_ENABLED
        const bool have_rangefinder = (dal.rangefinder() != nullptr && dal.rangefinder()->has_orientation(ROTATION_PITCH_270));
#else
        const bool have_rangefinder = false;
#endif
        if (!have_rangefinder) {
            hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "RangeFinder");
            return false;
        }
    }

    if (visualodom_required) {
        bool visualodom_available = false;
#if HAL_VISUALODOM_ENABLED
        auto *vo = AP::dal().visualodom();
        visualodom_available = vo && vo->enabled();
#endif
        if (!visualodom_available) {
            hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "VisualOdom");
            return false;
        }
    }

    if (wheelencoder_required && !dal.wheelencoder_enabled()) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "WheelEncoder");
        return false;
    }

    return true;
}

// return true if ext nav is enabled on any source
bool AP_NavEKF_Source::ext_nav_enabled(void) const
{
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
        const auto &src = _source_set[i];
        if (src.posxy == SourceXY::EXTNAV) {
            return true;
        }
        if (src.posz == SourceZ::EXTNAV) {
            return true;
        }
        if (src.velxy == SourceXY::EXTNAV) {
            return true;
        }
        if (src.velz == SourceZ::EXTNAV) {
            return true;
        }
        if (src.yaw == SourceYaw::EXTNAV) {
            return true;
        }
    }
    return false;
}

// return true if wheel encoder is enabled on any source
bool AP_NavEKF_Source::wheel_encoder_enabled(void) const
{
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
        const auto &src = _source_set[i];
        if (src.velxy == SourceXY::WHEEL_ENCODER) {
            return true;
        }
    }
    return false;
}

// returns active source set
uint8_t AP_NavEKF_Source::get_active_source_set() const
{
    return active_source_set;
}

// return true if GPS yaw is enabled on any source
bool AP_NavEKF_Source::gps_yaw_enabled(void) const
{
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
        const auto &src = _source_set[i];
        const SourceYaw yaw = SourceYaw(src.yaw.get());
        if (yaw == SourceYaw::GPS ||
            yaw == SourceYaw::GPS_COMPASS_FALLBACK) {
            return true;
        }
    }
    return false;
}
