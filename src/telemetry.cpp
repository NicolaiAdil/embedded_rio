#include "telemetry.h"

#include <Arduino.h>
#include <common/mavlink.h>

#undef B0
#undef B1
#undef B2
#undef B3

namespace telemetry {
namespace {

uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];

// Pack the upper triangle (21 floats) of a 6x6 covariance from a subset
// of rows/cols of P. r[i] is the P index for output row i; nan_col[i]
// blanks the i-th row and column.
void fillCov6(float out[21], const rio::Mat21& P,
              const int r[6], bool nan_col[6]) {
  int k = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++, k++) {
      if (nan_col[i] || nan_col[j]) {
        out[k] = NAN;
      } else {
        out[k] = P(r[i], r[j]);
      }
    }
  }
}

}  // namespace

void init() {
  Serial2.begin(921600);  // Telem1
}

void sendOdometry(const rio::NominalState& x, const rio::Mat21& P,
                  float t_sec, int8_t quality) {
  // Rotate filter-internal frame to PX4's NED/FRD output.
  static const rio::Quat q_t(0.0f, 0.7071067811865475f, 0.7071067811865475f, 0.0f);

  const rio::Vec3 p_out  = q_t * x.p_WI;
  const rio::Vec3 v_body = q_t * (x.q_WI.inverse() * x.v_WI);
  const rio::Quat q_out  = q_t * x.q_WI * q_t.inverse();

  mavlink_odometry_t odom{};
  odom.time_usec      = (uint64_t)(t_sec * 1e6f);
  odom.frame_id       = MAV_FRAME_LOCAL_NED;
  odom.child_frame_id = MAV_FRAME_BODY_FRD;

  odom.x = p_out.x();
  odom.y = p_out.y();
  odom.z = p_out.z();

  odom.q[0] = q_out.w();
  odom.q[1] = q_out.x();
  odom.q[2] = q_out.y();
  odom.q[3] = q_out.z();

  odom.vx = v_body.x();
  odom.vy = v_body.y();
  odom.vz = v_body.z();

  odom.quality = quality;

  odom.rollspeed  = NAN;
  odom.pitchspeed = NAN;
  odom.yawspeed   = NAN;

  // pose_covariance: upper triangle of [pos(0:3), att(9:12)] from P.
  {
    const int r[6]       = {0, 1, 2, 9, 10, 11};
    bool      nan_col[6] = {false, false, false, false, false, false};
    fillCov6(odom.pose_covariance, P, r, nan_col);
  }

  // velocity_covariance: upper triangle of [vel(3:6), ang_vel=NaN].
  {
    const int r[6]       = {3, 4, 5, 0, 0, 0};
    bool      nan_col[6] = {false, false, false, true, true, true};
    fillCov6(odom.velocity_covariance, P, r, nan_col);
  }

  odom.reset_counter  = 0;
  odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

  mavlink_message_t msg;
  mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
  uint16_t len = mavlink_msg_to_send_buffer(mav_buf, &msg);
  Serial2.write(mav_buf, len);
}

}  // namespace telemetry
