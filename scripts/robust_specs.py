from stlpy.STL import Always

def load_specs(num_waypoints):
    return [
        Always(0, num_waypoints-1, 'x >= -0.5'),
        Always(0, num_waypoints-1, 'y <= -0.2'),
        ...
    ]
