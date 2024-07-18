def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    reward = 1e-3
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering_angle = abs(params['steering_angle']) # Only need the absolute steering angle
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']

    # Calculate 3 marks that are farther and father away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward += 5
    elif distance_from_center <= marker_2:
        reward += 1.5
    elif distance_from_center <= marker_3:
        reward += 0.3

    if -5 < steering_angle < 5:
        if speed > 3.0:
            reward += 5.0
        elif speed > 2:
            reward += 2.5
    elif steering_angle < -15 or steering_angle > 15:
        if speed < 1.8:
            reward += 3.0
        elif speed < 2.2:
            reward += 1.5

    if steps > 5:
        reward += 10 * progress / steps

    return float(reward)
