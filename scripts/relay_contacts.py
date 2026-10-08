"""Persistent actor contacts aggregated from PhysX collider-pair events."""


class ActorPathCache:
    """Actor IDs are stable within a scene; discard on scene/reset boundaries."""
    def __init__(self, decode):
        self.decode = decode
        self.paths = {}

    def __call__(self, actor):
        key = int(actor)
        if key not in self.paths:
            self.paths[key] = str(self.decode(actor))
        return self.paths[key]

    def clear(self):
        self.paths.clear()


class ContactTracker:
    def __init__(self):
        self.actors = {}
        self.pairs = {}

    def clear(self):
        self.actors.clear()
        self.pairs.clear()

    def update(self, actor0, actor1, collider0, collider1, *, lost=False):
        key = tuple(sorted((collider0, collider1)))
        for box, other in ((actor0, actor1), (actor1, actor0)):
            if '/Crates/' not in box and '/Robots/' not in box:
                continue
            pairs = self.pairs.setdefault((box, other), set())
            actors = self.actors.setdefault(box, set())
            if lost:
                pairs.discard(key)
            else:
                pairs.add(key)
            if pairs:
                actors.add(other)
            else:
                actors.discard(other)
                self.pairs.pop((box, other), None)
