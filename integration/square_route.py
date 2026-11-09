"""Simulation-only waypoint controller; never supplies pose to the estimator."""
import math
import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class SquareRoute:
    def __init__(self, side_length, speed):
        self.side_length, self.speed = side_length, speed
        self.origin = None
        self.waypoints = None
        self.heading = None
        self.side = 0
        self.phase = "drive"
        self.deadline = 0.
        self.corners = []
        self.heading_integral = 0.
        self.previous_time = None

    @property
    def done(self): return self.phase == "done"

    def update(self, t, position, yaw):
        xy = np.asarray(position, dtype=float)
        if self.origin is None:
            self.origin, self.heading = xy.copy(), float(yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            self.rotation = np.array([[c, -s], [s, c]])
            square = self.side_length * np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
            self.waypoints = square @ self.rotation.T + xy
        dt = 0.02 if self.previous_time is None else max(0., min(.1, t-self.previous_time))
        self.previous_time = t
        if self.done: return np.zeros(3)

        if self.phase == "corner_settle":
            if t < self.deadline: return np.zeros(3)
            self.corners.append({"side": self.side+1, "time_s": float(t),
                                 "target_world_xy": self.waypoints[self.side+1].tolist(),
                                 "actual_world_xy": xy.tolist(), "yaw_rad": float(yaw)})
            print(f"Square corner {self.side+1}/4: t={t:.2f}s, target error="
                  f"{np.linalg.norm(xy-self.waypoints[self.side+1]):.3f}m", flush=True)
            self.side += 1
            self.heading_integral = 0.
            if self.side == 4:
                self.phase = "done"
                return np.zeros(3)
            self.phase = "turn"

        desired_heading = self.heading + self.side*math.pi/2
        if self.phase == "turn":
            error = wrap(desired_heading-yaw)
            if abs(error) < .035:
                self.phase, self.deadline = "turn_settle", t+.8
                return np.zeros(3)
            rate = math.copysign(max(.14, min(.5, 1.4*abs(error))), error)
            return np.array([0., 0., rate])
        if self.phase == "turn_settle":
            if t < self.deadline: return np.zeros(3)
            self.phase = "drive"

        target = self.waypoints[self.side+1]
        distance = float(np.linalg.norm(target-xy))
        direction = np.array([math.cos(desired_heading), math.sin(desired_heading)])
        normal = np.array([-direction[1], direction[0]])
        remaining = float((target-xy) @ direction)
        cross_track = float((xy-self.waypoints[self.side]) @ normal)
        if distance < .03 or (remaining < .01 and distance < .07):
            self.phase, self.deadline = "corner_settle", t+1.5
            return np.zeros(3)
        # Follow the side with a 0.6 m lookahead; after a small overshoot,
        # heading toward the actual target prevents an unbounded straight run.
        lookahead = max(.15, min(.6, remaining))
        heading = desired_heading - math.atan2(cross_track, lookahead)
        if remaining < 0: heading = math.atan2(target[1]-xy[1], target[0]-xy[0])
        error = wrap(heading-yaw)
        self.heading_integral = float(np.clip(self.heading_integral+error*dt, -.8, .8))
        rate = float(np.clip(1.7*error+.2*self.heading_integral, -.6, .6))
        speed = min(self.speed, max(.13, .45*distance))
        if abs(error) > .8: speed = 0.
        return np.array([speed, 0., rate])

    def describe(self):
        return {"side_length_m": self.side_length, "nominal_perimeter_m": 4*self.side_length,
                "route_completed": self.done, "final_phase": self.phase,
                "route_pose_source": "MuJoCo position and IMU yaw for waypoint control only",
                "origin_world_xy": None if self.origin is None else self.origin.tolist(),
                "waypoints_world_xy": None if self.waypoints is None else self.waypoints.tolist(),
                "corners": self.corners}
