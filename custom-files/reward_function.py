import math
from functools import cached_property


class Reward:
    DEFAULT_REWARD = 1e-3

    def __init__(self):
        self.prev_steps = None

    def set_prev_params_to_default(self):
        self.prev_steps = None
        self.intermediate_progress_reward = [0] * 12

    def read_params(self, params):
        # Read input parameters
        self.distance_from_center = max(0.001, params['distance_from_center'])
        self.track_width = params['track_width']
        self.steering_angle = abs(params['steering_angle']) # Only need the absolute steering angle
        self.speed = params['speed']
        self.steps = params['steps']
        self.progress = params['progress']
        self.is_offtrack = params['is_offtrack']
        self.closest_waypoints = params['closest_waypoints']
        self.waypoints = params['waypoints']
        self.heading = params['heading']
        self.all_wheels_on_track = params['all_wheels_on_track']
        self.reward = self.DEFAULT_REWARD
        self.direction_diff = self.direction_difference_calculator()

    def direction_difference_calculator(self):
        # Calculate the direction of the center line based on the closest waypoints
        next_point = self.waypoints[self.closest_waypoints[1]]
        prev_point = self.waypoints[self.closest_waypoints[0]]

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
        # Convert to degree
        track_direction = math.degrees(track_direction)

        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - self.heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        return direction_diff

    @property
    def is_direction_above_threshold(self):
        # Penalize the reward if the difference is too large
        DIRECTION_THRESHOLD = 15.0
        if self.direction_diff > DIRECTION_THRESHOLD:
            return True
        return False

    @property
    def is_penalized(self):
        if self.is_offtrack \
           or self.is_direction_above_threshold \
           or not self.all_wheels_on_track:
            return True
        return False

    def distance_reward(self):
        DISTANCE_THRESHOLD = 0.45 * self.track_width
        if self.distance_from_center <= DISTANCE_THRESHOLD:
            self.reward += 5 - (10 * self.distance_from_center / self.track_width)

        # Give higher reward if the car is closer to center line and vice versa
        # if self.distance_from_center <= 0.1 * DISTANCE_THRESHOLD:
        #     multiplier = 5
        # elif self.distance_from_center <= 0.25 * DISTANCE_THRESHOLD:
        #     multiplier = 2
        # elif self.distance_from_center <= 0.5 * DISTANCE_THRESHOLD:
        #     multiplier = 1
        # else:
        #     multiplier = self.DEFAULT_REWARD

        # distance_reward = max(0, 1 - (self.distance_from_center / DISTANCE_THRESHOLD))
        # self.reward += distance_reward * multiplier

    def progress_reward(self):
        MAX_PROGRESS_REWARD_MULTIPLIER = 5
        TIME_THRESHOLD = 15.0
        progress_checkpoint = int(self.progress // 10)
        if self.intermediate_progress_reward[progress_checkpoint] == 0:
            self.intermediate_progress_reward[progress_checkpoint] = MAX_PROGRESS_REWARD_MULTIPLIER * ((self.progress * TIME_THRESHOLD / 100) / (self.steps/15))
            self.reward += self.intermediate_progress_reward[progress_checkpoint]
        else:
            self.reward += self.DEFAULT_REWARD

    @cached_property
    def fast_indexes(self):
        return [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
                26, 27, 28, 29, 30, 31, 32, 33, 41, 42, 43, 44, 45, 46, 47, 48, 55, 56, 57, 58, 59, 60, 61, 62,
                63, 64, 65, 66, 67, 68, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 122, 123, 124, 125, 126]

    @cached_property
    def medium_indexes(self):
        return [49, 50, 51, 52, 53, 54, 69, 70, 71, 72, 73, 74, 75, 81, 82, 83, 84, 85, 86, 87, 88, 98, 99, 100, 101, 102, 103]

    @cached_property
    def slow_indexes(self):
        return [34, 35, 36, 37, 38, 39, 40, 76, 77, 78, 79, 80, 89, 90, 91, 92, 93, 94, 95, 96, 97, 104, 105, 106, 107]

    def reward_function(self, params):
        if self.prev_steps is None or self.steps < self.prev_steps:
            self.set_prev_params_to_default()

        self.read_params(params)

        if self.is_penalized:
            return self.reward

        self.distance_reward()
        self.progress_reward()

        self.prev_steps = self.steps

        return float(self.reward)


reward_calc = Reward()


def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    return reward_calc.reward_function(params)
