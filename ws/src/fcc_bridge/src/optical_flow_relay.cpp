// Copyright 2026 Jun Hong Lee. All rights reserved.
// Proprietary — Swarm Search Drone Mission Computer (SDD-MC-001 §3, ICD-MC-001 §3).

#include "fcc_bridge/optical_flow_relay.hpp"

#include <cmath>

namespace fcc_bridge
{

void OpticalFlowRelay::handle_frame(const mavlink_message_t & msg)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
      handle_optical_flow_rad(msg);
      break;
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
      handle_distance_sensor(msg);
      break;
    case MAVLINK_MSG_ID_ALTITUDE:
      handle_altitude(msg);
      break;
    default:
      break;
  }
}

void OpticalFlowRelay::handle_optical_flow_rad(const mavlink_message_t & msg)
{
  mavlink_optical_flow_rad_t flow{};
  mavlink_msg_optical_flow_rad_decode(&msg, &flow);

  if (flow.integration_time_us == 0) {
    return;  // avoid div-by-zero on a malformed/empty integration window
  }

  // distance: prefer the frame's own paired reading; fall back to the most
  // recent standalone DISTANCE_SENSOR frame otherwise. -1 means unavailable
  // in both MAVLink's and PX4's internal convention.
  const float range = (flow.distance >= 0.0F) ? flow.distance : last_distance_sensor_m_;
  if (range < 0.0F) {
    return;  // no height reference yet -- can't scale angle to velocity
  }

  // Angle-rate -> body-frame velocity, replicated exactly from PX4's own
  // VehicleOpticalFlow.cpp (src/modules/sensors/vehicle_optical_flow/
  // VehicleOpticalFlow.cpp lines ~281-291, confirmed against the real
  // v1.14.4 tag originally, re-confirmed byte-for-byte at v1.17.0 once
  // this project's target firmware moved): PX4 already applies
  // SENS_FLOW_ROT to integrated_x/y and
  // integrated_xgyro/ygyro before this MAVLink stream ever sees them
  // (OPTICAL_FLOW_RAD.hpp publishes straight from the post-rotation
  // `vehicle_optical_flow` uORB topic), so no additional rotation is needed
  // here -- only the sign flip and gyro-compensation PX4 itself performs
  // when deriving vehicle_optical_flow_vel from the same fields.
  const float flow_dt_s = static_cast<float>(flow.integration_time_us) * 1e-6F;
  const float flow_x_rad = -flow.integrated_x;
  const float flow_y_rad = -flow.integrated_y;
  const float gyro_x = -flow.integrated_xgyro;
  const float gyro_y = -flow.integrated_ygyro;
  const float flow_comp_x = flow_x_rad - gyro_x;
  const float flow_comp_y = flow_y_rad - gyro_y;

  std::scoped_lock lock(mtx_);
  sample_.vx = -range * flow_comp_y / flow_dt_s;  // body FRD X (forward)
  sample_.vy = range * flow_comp_x / flow_dt_s;   // body FRD Y (right)
  sample_.ground_distance = range;
  sample_.quality = flow.quality;
  ++sample_.seq;
  sample_.valid = true;
}

void OpticalFlowRelay::handle_distance_sensor(const mavlink_message_t & msg)
{
  mavlink_distance_sensor_t dist{};
  mavlink_msg_distance_sensor_decode(&msg, &dist);

  last_distance_sensor_m_ = static_cast<float>(dist.current_distance) * 1e-2F;  // cm -> m
}

void OpticalFlowRelay::handle_altitude(const mavlink_message_t & msg)
{
  mavlink_altitude_t alt{};
  mavlink_msg_altitude_decode(&msg, &alt);

  // PX4 sends NaN until vehicle_air_data has published at least once
  // (streams/ALTITUDE.hpp) -- no baro yet, not an altitude of zero.
  if (!std::isfinite(alt.altitude_monotonic)) {
    return;
  }

  std::scoped_lock lock(mtx_);
  baro_.alt_m = alt.altitude_monotonic;
  ++baro_.seq;
  baro_.valid = true;
}

BaroAltitudeSample OpticalFlowRelay::get_baro_altitude() const
{
  std::scoped_lock lock(mtx_);
  return baro_;
}

OpticalFlowSample OpticalFlowRelay::get_sample() const
{
  std::scoped_lock lock(mtx_);
  return sample_;
}

}  // namespace fcc_bridge
