class Obstacle:

    def __init__(self, obs_id, x, y, z, diameter, lifetime):
        self.id = obs_id
        self.x = x
        self.y = y
        self.z = z
        self.diameter = diameter
        self.lifetime = lifetime

    def __str__(self):
        return 'id: %d, x: %.2f, y: %.2f, z: %.2f, diameter(mm): %.2f, lifetime: %d' % (self.id,
                                                                                        self.x,
                                                                                        self.y,
                                                                                        self.z,
                                                                                        self.diameter,
                                                                                        self.lifetime)

