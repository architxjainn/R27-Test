/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate. Implement
 * drive_to_target() so the simulated differential-drive rover reaches the
 * target using valid left and right wheel velocities.
 */

#include <math.h>
#include <stdbool.h>

#define PI_F 3.14159265358979323846f

#define WHEEL_RADIUS 0.15f
#define WHEEL_SEPARATION 0.77f
#define MAX_LINEAR_VELOCITY 1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY 10.0f
#define HEADING_GAIN 1.25f

#define TARGET_TOLERANCE 0.10f
#define DRIVE_DT_SECONDS 0.02f
#define MAX_DRIVE_STEPS 6000

/*
 * Latitude and longitude are normalized local simulation coordinates measured
 * in metres. Latitude is the north axis and longitude is the east axis. The
 * differential-drive rover is planar, so altitude is received but not changed.
 */
struct coordinate {
  float latitude;
  float longitude;
  float altitude;
};

/* Heading is in radians: zero points east and positive rotation is CCW. */
struct rover_state {
  struct coordinate position;
  float heading_rad;
};

struct wheel_velocity {
  float left;
  float right;
};

enum drive_status {
  DRIVE_REACHED_TARGET = 0,
  DRIVE_INVALID_INPUT = -1,
  DRIVE_INVALID_COMMAND = -2,
  DRIVE_MAX_STEPS_EXCEEDED = -3
};

/* Provided simulator helpers. Candidates should not modify these functions. */
static float normalize_angle(float angle);
static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity);

/*
 * Candidate task
 * --------------
 * The target argument contains the coordinate produced by the upstream comms
 * and decoding stages. Use the rover's current position and heading to produce
 * separate left and right wheel velocities on every iteration.
 *
 * Requirements:
 *   - reject invalid inputs safely;
 *   - stop after reaching the target within TARGET_TOLERANCE;
 *   - keep both wheel velocities within MAX_WHEEL_VELOCITY;
 *   - handle targets in every direction and heading wraparound correctly;
 *   - return DRIVE_MAX_STEPS_EXCEEDED if the rover does not converge; and
 *   - guarantee that the function terminates.
 *
 * Implement the function body below. The simulator helpers are already provided
 * and may be called from your solution. Path planning and PID are not required.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                  const struct coordinate *target) {
  // 1. Validate inputs
  if (rover == NULL || target == NULL) {
    return DRIVE_INVALID_INPUT;
  }
  if (!isfinite(rover->heading_rad) ||
      !isfinite(rover->position.latitude) ||
      !isfinite(rover->position.longitude) ||
      !isfinite(target->latitude) ||
      !isfinite(target->longitude) ||
      !isfinite(target->altitude)) {
    return DRIVE_INVALID_INPUT;
  }

  int steps = 0;

  while (steps < MAX_DRIVE_STEPS) {
    // Compute vector to target
    float dx = target->longitude - rover->position.longitude;
    float dy = target->latitude - rover->position.latitude;
    float distance = sqrtf(dx * dx + dy * dy);

    // Check if target reached
    if (distance < TARGET_TOLERANCE) {
      struct wheel_velocity stop = {0.0f, 0.0f};
      apply_wheel_velocities(rover, stop);
      return DRIVE_REACHED_TARGET;
    }

    // Compute desired heading
    float target_heading = atan2f(dy, dx);
    float heading_error = normalize_angle(target_heading - rover->heading_rad);

    // Linear velocity: full speed when far, slow down near target
    float linear_vel = MAX_LINEAR_VELOCITY;
    if (distance < 2.0f) {
      linear_vel = MAX_LINEAR_VELOCITY * (distance / 2.0f);
      if (linear_vel < 0.05f) linear_vel = 0.05f; // minimum for controllability
    }

    // Angular velocity: proportional to heading error
    float angular_vel = HEADING_GAIN * heading_error;
    if (angular_vel > MAX_ANGULAR_VELOCITY)
      angular_vel = MAX_ANGULAR_VELOCITY;
    else if (angular_vel < -MAX_ANGULAR_VELOCITY)
      angular_vel = -MAX_ANGULAR_VELOCITY;

    // Convert to wheel velocities
    struct wheel_velocity vel;
    vel.left  = (linear_vel / WHEEL_RADIUS) -
                (angular_vel * WHEEL_SEPARATION) / (2.0f * WHEEL_RADIUS);
    vel.right = (linear_vel / WHEEL_RADIUS) +
                (angular_vel * WHEEL_SEPARATION) / (2.0f * WHEEL_RADIUS);

    // Clamp wheel velocities to MAX_WHEEL_VELOCITY
    float max_abs = fmaxf(fabsf(vel.left), fabsf(vel.right));
    if (max_abs > MAX_WHEEL_VELOCITY) {
      float scale = MAX_WHEEL_VELOCITY / max_abs;
      vel.left  *= scale;
      vel.right *= scale;
    }

    // Apply velocities
    if (!apply_wheel_velocities(rover, vel)) {
      return DRIVE_INVALID_INPUT;
    }

    steps++;
  }

  return DRIVE_MAX_STEPS_EXCEEDED;
}

static float normalize_angle(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity) {
  if (!isfinite(velocity.left) || !isfinite(velocity.right) ||
      fabsf(velocity.left) > MAX_WHEEL_VELOCITY ||
      fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
    return false;
  }

  const float linear_velocity =
      WHEEL_RADIUS * (velocity.left + velocity.right) / 2.0f;
  const float angular_velocity =
      WHEEL_RADIUS * (velocity.right - velocity.left) / WHEEL_SEPARATION;

  rover->heading_rad = normalize_angle(
      rover->heading_rad + angular_velocity * DRIVE_DT_SECONDS);
  rover->position.longitude +=
      linear_velocity * cosf(rover->heading_rad) * DRIVE_DT_SECONDS;
  rover->position.latitude +=
      linear_velocity * sinf(rover->heading_rad) * DRIVE_DT_SECONDS;

  return true;
}
