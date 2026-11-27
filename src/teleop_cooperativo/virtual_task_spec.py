"""Shared visual and collision shapes for the virtual tabletop objects."""

# All dimensions are metres; each object is about 0.10 m tall and 0.10 kg.
OBJECTS = (
    {'name': 'Luna', 'color': (0.87, 0.27, 0.31), 'body': 'cylinder', 'head': 'sphere'},
    {'name': 'Cubo', 'color': (0.22, 0.46, 0.83), 'body': 'box', 'head': 'box'},
    {'name': 'Sol', 'color': (0.96, 0.65, 0.15), 'body': 'capsule', 'head': 'sphere'},
    {'name': 'Orejas', 'color': (0.30, 0.66, 0.43), 'body': 'cylinder', 'head': 'ears'},
    {'name': 'Casco', 'color': (0.58, 0.37, 0.75), 'body': 'box', 'head': 'cylinder'},
)
INITIAL = tuple((x, -0.26, 0.052) for x in (0.19, 0.31, 0.43, 0.55, 0.67))
GOAL_HEIGHTS = (0.0, 0.04, 0.08, 0.12, 0.16)
GOALS = tuple((x, 0.27, height + 0.001) for x, height in zip((0.19, 0.31, 0.43, 0.55, 0.67), GOAL_HEIGHTS))


def _parts(spec):
    body = {'shape': spec['body'], 'center': (0, 0, -0.018), 'radius': 0.018, 'height': 0.064, 'half': (0.018, 0.018, 0.032)}
    # A capsule includes hemispherical ends, so its cylindrical part is shorter.
    if spec['body'] == 'capsule':
        body = {**body, 'height': 0.028}
    head = {'shape': 'box' if spec['head'] == 'box' else 'cylinder' if spec['head'] == 'cylinder' else 'sphere',
            'center': (0, 0, 0.032), 'radius': 0.017, 'height': 0.034, 'half': (0.017, 0.017, 0.017)}
    parts = [body, head]
    if spec['head'] == 'ears':
        parts.extend({'shape': 'sphere', 'center': (x, 0, 0.043), 'radius': 0.006, 'height': 0.0, 'half': (0, 0, 0)} for x in (-0.019, 0.019))
    return parts
