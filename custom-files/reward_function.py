import math


class Reward:
    DEFAULT_REWARD = 1e-4
    CAR_DIRECTION_DIFFERENCE_THRESHOLD = 5
    CAR_DIRECTION_DIFFERENCE_REWARD_FACTOR = 1.2
    HEADING_DIRECTION_DIFFERENCE_THRESHOLD = 15
    HEADING_FACTOR = 1.0
    STEERING_THRESHOLD = 15
    STEERING_FACTOR = 0.4
    DISTANCE_FACTOR = 0.8
    SPEED_FACTOR = 1.0
    MIN_SPEED = 1.5
    MAX_SPEED = 4.0

    def __init__(self):
        self.reset_prev_params()

    def reset_prev_params(self):
        self.prev_steps = None
        self.intermediate_progress_reward = [0] * 12

    def read_params(self, params):
        # Read input parameters
        self.distance_from_center = params['distance_from_center']
        self.track_width = params['track_width']
        self.steering_angle = params['steering_angle']
        self.speed = params['speed']
        self.steps = params['steps']
        self.progress = params['progress']
        self.is_offtrack = params['is_offtrack']
        self.closest_waypoints = params['closest_waypoints']
        self.waypoints = params['waypoints']
        self.heading = params['heading']
        self.all_wheels_on_track = params['all_wheels_on_track']
        self.reward = self.DEFAULT_REWARD
        self.total_waypoints = len(self.waypoints)

    @property
    def is_penalized(self):
        if self.is_offtrack \
           or self.is_car_direction_diff_above_threshold \
           or self.is_heading_direction_diff_above_threshold \
           or not self.all_wheels_on_track:
            return True
        return False

    def direction_difference(self, direction):
        # Calculate the direction of the center line based on the closest waypoints
        next_point = self.waypoints[(self.closest_waypoints[1] + 1) % self.total_waypoints]
        prev_point = self.waypoints[self.closest_waypoints[0]]

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
        # Convert to degree
        track_direction = math.degrees(track_direction)

        # Calculate the difference between the track direction and the car's intended direction
        direction_diff = abs(track_direction - direction)
        if direction_diff > 180:
            direction_diff = abs(360 - direction_diff)

        return direction_diff

    @property
    def car_direction_diff(self):
        return self.direction_difference(self.heading + self.steering_angle)

    @property
    def is_car_direction_diff_above_threshold(self):
        if self.car_direction_diff > self.CAR_DIRECTION_DIFFERENCE_THRESHOLD:
            return True
        return False

    @property
    def heading_direction_diff(self):
        return self.direction_difference(self.heading)

    @property
    def is_heading_direction_diff_above_threshold(self):
        if self.heading_direction_diff > self.HEADING_DIRECTION_DIFFERENCE_THRESHOLD:
            return True
        return False

    def distance_reward(self):
        DISTANCE_THRESHOLD = 0.45 * self.track_width
        if self.distance_from_center <= DISTANCE_THRESHOLD:
            self.reward += (1 - (2 * self.distance_from_center / self.track_width)) * self.DISTANCE_FACTOR

    def direction_reward(self):
        self.reward += ((self.CAR_DIRECTION_DIFFERENCE_THRESHOLD - self.car_direction_diff) / self.CAR_DIRECTION_DIFFERENCE_THRESHOLD) * self.CAR_DIRECTION_DIFFERENCE_REWARD_FACTOR

    def heading_reward(self):
        self.reward += ((self.HEADING_DIRECTION_DIFFERENCE_THRESHOLD - self.heading_direction_diff) / self.HEADING_DIRECTION_DIFFERENCE_THRESHOLD) * self.HEADING_FACTOR

    def progress_reward(self):
        if int(self.progress) == 100:
            self.reward += 100

    def speed_reward(self):
        if abs(self.steering_angle) < 7:
            if self.speed >= 3.0:
                self.reward += (self.speed / self.MAX_SPEED) * self.SPEED_FACTOR
        elif abs(self.steering_angle) < 12:
            if self.speed >= 3.0 and self.speed <= 3.6:
                self.reward += (self.speed / self.MAX_SPEED) * self.SPEED_FACTOR
        elif abs(self.steering_angle) < 18:
            if self.speed >= 2.3 and self.speed <= 3.0:
                self.reward += self.SPEED_FACTOR
        elif abs(self.steering_angle) < 25:
            if self.speed >= 1.8 and self.speed <= 2.5:
                self.reward += self.SPEED_FACTOR
        else:
            if self.speed <= 2.0:
                self.reward += (1.375 - (self.speed / self.MAX_SPEED)) * self.SPEED_FACTOR

    def reward_function(self, params):
        self.read_params(params)

        if self.prev_steps is None or self.steps < self.prev_steps:
            self.reset_prev_params()

        if self.is_penalized:
            return self.reward

        self.distance_reward()
        self.direction_reward()
        self.heading_reward()
        self.progress_reward()
        self.speed_reward()
        self.prev_steps = self.steps

        return self.reward


reward_obj = Reward()


def reward_function(params):
    return reward_obj.reward_function(params)
