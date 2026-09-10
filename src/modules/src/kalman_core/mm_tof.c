/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_tof.h"
#include "param.h"
#include "log.h"

// ---- Innovation gate for the down-range (ToF) update ----
// Rejects brief, physically-impossible jumps in the ToF reading (e.g. an obstacle
// or ledge passing under the drone) so the EKF's z COASTS on the accelerometer
// prediction through the glitch instead of lurching. Because z stays ~correct
// during the glitch, the reading also recovers smoothly on the far edge (small
// innovation) -> no jump AND no drop. A change that PERSISTS past tofGateHold
// samples is treated as a real terrain step and accepted. Set tofGate=0 to disable.
// 2026-09-10 (flight 19:55): gating against the EKF's own prediction let a sill CREEP in -- the ToF cone
// blends sill and floor on the way in, so the reading ramps down in steps below the gate, each accepted
// one drags z along, and once z is wrong the true floor is rejected for the whole hold. The reference is
// therefore the LAST ACCEPTED reading, with an allowance that grows at a physical vertical speed:
//   |measured - lastAccepted| > gate + rate * (time since the last accept)  -> reject (up to hold)
// A ramp is rejected (a real descent never moves 0.15 m in 25 ms), and the floor is accepted the instant
// it is back, since it matches the reference.
static float    tofGate     = 0.10f;  // [m] base allowance vs the last accepted reading (0 = off)
static float    tofRate     = 0.30f;  // [m/s] the allowance grows at this rate while rejecting
static uint16_t tofGateHold = 40;     // accept after this many consecutive rejects (~real step; 40 = 1 s)
static float    tofRef      = -1.0f;  // last accepted reading [m]
static uint32_t tofRefMs    = 0;      // ...and when
static uint16_t tofRejects  = 0;      // running count of consecutive rejects
static uint8_t  tofGated    = 0;      // 1 = last ToF update was rejected (for logging)
static float    tofInnov    = 0.0f;   // last innovation [m] (for tuning tofGate)

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    float predictedDistance = this->S[KC_STATE_Z] / cosf(angle);
    float measuredDistance = tof->distance; // [m]

    /*
    The sensor model (Pg.95-96, https://lup.lub.lu.se/student-papers/search/publication/8905295)
    
    h = z/((R*z_b).z_b) = z/cos(alpha)
    
    Here,
    h (Measured variable)[m] = Distance given by TOF sensor. This is the closest point from any surface to the sensor in the measurement cone
    z (Estimated variable)[m] = THe actual elevation of the crazyflie
    z_b = Basis vector in z direction of body coordinate system
    R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be stationary
    alpha = angle between [line made by measured point <---> sensor] and [the intertial z-axis] 
    */

    h[KC_STATE_Z] = 1 / cosf(angle); // This just acts like a gain for the sensor model. Further updates are done in the scalar update function below

    // Innovation gate: reject a brief implausible jump (obstacle glitch), but
    // accept a change that persists (real terrain step). See note at top.
    float innovation = measuredDistance - predictedDistance;
    tofInnov = innovation;
    uint32_t now = tof->timestamp;                       // ticks = ms
    if (tofRef < 0.0f) { tofRef = measuredDistance; tofRefMs = now; }
    float allowed = tofGate + tofRate * (float)(now - tofRefMs) * 0.001f;
    if (tofGate > 0.0001f && fabsf(measuredDistance - tofRef) > allowed && tofRejects < tofGateHold) {
      tofRejects++;    // glitch -> skip fusion; z coasts on the accel prediction
      tofGated = 1;
    } else {
      tofRejects = 0;  // plausible, or a persisted real change -> accept & re-converge
      tofGated = 0;
      tofRef = measuredDistance; tofRefMs = now;
      kalmanCoreScalarUpdate(this, &H, innovation, tof->stdDev);
    }
  }
}

/**
 * Down-range (ToF) innovation gate — rejects obstacle-induced height glitches
 * before they reach the EKF. Set n6/height glitches vanish without a lurch.
 */
PARAM_GROUP_START(tofGate)
/**
 * @brief Base allowance [m] vs the LAST ACCEPTED reading. 0 = off. (default 0.10)
 */
PARAM_ADD(PARAM_FLOAT, gate, &tofGate)
/**
 * @brief The allowance grows at this vertical rate [m/s] while rejecting. (default 0.30)
 */
PARAM_ADD(PARAM_FLOAT, rate, &tofRate)
/**
 * @brief Accept a gated change after this many consecutive rejects (real terrain step). (default 40 = 1 s)
 */
PARAM_ADD(PARAM_UINT16, hold, &tofGateHold)
PARAM_GROUP_STOP(tofGate)

LOG_GROUP_START(tofGate)
/**
 * @brief 1 = the most recent ToF update was rejected by the gate
 */
LOG_ADD(LOG_UINT8, gated, &tofGated)
/**
 * @brief Most recent ToF innovation (measured - predicted) [m]
 */
LOG_ADD(LOG_FLOAT, innov, &tofInnov)
/**
 * @brief Consecutive rejects (climbs to tofGate.hold then forces accept)
 */
LOG_ADD(LOG_UINT16, rej, &tofRejects)
/**
 * @brief The reference: last accepted ToF reading [m]
 */
LOG_ADD(LOG_FLOAT, ref, &tofRef)
LOG_GROUP_STOP(tofGate)
