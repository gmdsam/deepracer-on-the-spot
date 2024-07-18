import math


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
        DIRECTION_THRESHOLD = 30.0
        if self.direction_diff > DIRECTION_THRESHOLD:
            return True
        return False

    def distance_reward(self):
        DISTANCE_THRESHOLD = 0.5 * self.track_width

        # Give higher reward if the car is closer to center line and vice versa
        if self.distance_from_center <= 0.1 * DISTANCE_THRESHOLD:
            multiplier = 5
        elif self.distance_from_center <= 0.25 * DISTANCE_THRESHOLD:
            multiplier = 2
        elif self.distance_from_center <= 0.5 * DISTANCE_THRESHOLD:
            multiplier = 1
        else:
            multiplier = 1e-3

        distance_reward = max(0, 1 - (self.distance_from_center / DISTANCE_THRESHOLD))
        return distance_reward * multiplier

    def progress_reward(self):
        MAX_PROGRESS_REWARD = 10
        TIME_THRESHOLD = 12.0
        progress_checkpoint = int(self.progress // 10)
        if self.intermediate_progress_reward[progress_checkpoint] == 0:
            self.intermediate_progress_reward[progress_checkpoint] = MAX_PROGRESS_REWARD * ((self.progress * TIME_THRESHOLD / 100) / (self.steps/15)) * progress_checkpoint
            return self.intermediate_progress_reward[progress_checkpoint]
        return self.DEFAULT_REWARD

    def reward_function(self, params):
        self.read_params(params)
        reward = self.DEFAULT_REWARD

        if self.prev_steps is None or self.steps < self.prev_steps:
            self.set_prev_params_to_default()

        if self.is_offtrack or self.is_direction_above_threshold:
            return reward

        reward += self.distance_reward()
        reward += self.progress_reward()

        self.prev_steps = self.steps

        return float(reward)


reward_calc = Reward()


def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    return reward_calc.reward_function(params)
