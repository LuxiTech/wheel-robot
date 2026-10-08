"""Deterministic relay inventory and transactional handoff, independent of Isaac.

Physical occupancy and logical ownership are deliberately separate. A lock is
released only by a confirmed completion; timeouts never turn occupied slots empty.
"""
from dataclasses import dataclass
from relay_layout import slot_position, pallet_yaw


class RelayFault(RuntimeError):
    pass


def source_order(initial):
    # Pickup ranks in rotated top view (back wall above): [3,2] / [5,6] / [4,1].
    priority = {slot: rank for rank, slot in enumerate(
        ((0, 0), (0, 2), (1, 2), (1, 0), (1, 1), (0, 1)))}
    return sorted(range(len(initial)), key=lambda i: (
        -initial[i]['layer'], priority[(initial[i]['row'], initial[i]['column'])]))


def source_handoff_slot(scene, spec):
    """A always deposits left; the pusher transfers left to right."""
    return 0


def source_handoff_far(scene, spec):
    """Choose the rear/front approach from source world Y, not deposit slot."""
    return spec['position'][1] > sum(slot[1] for slot in scene['handoff']['slots']) / len(scene['handoff']['slots'])


def target_order(scene):
    stack = scene['source_stack']
    if (stack['rows'], stack['columns']) != (2, 3):
        raise ValueError('Relay strategy requires two rows and three columns')
    x, y = scene['pallet']['target']
    sx, sy, sz = scene['crate']['size']
    return [dict(layer=k, row=r, column=c,
                 position=slot_position(scene, 'target', c, r, k), yaw=pallet_yaw(scene))
            for k in range(stack['layers'])
            for c, r in ((1, 1), (1, 0), (0, 1), (0, 0), (2, 1), (2, 0))]


@dataclass(frozen=True)
class HandoffObservation:
    left: tuple = ()
    right: tuple = ()
    stable: frozenset = frozenset()
    arms_clear: bool = False
    pusher_home: bool = False
    valid: bool = True


class HandoffCoordinator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.owner = None
        self.box = None
        self.action = None
        self.deposit_slot = None
        self.sequence = 0
        self.completed = set()
        self.fault = None
        self.observation = HandoffObservation(valid=False)
        self.transfers = 0

    def observe(self, observation):
        self.observation = observation
        if len(observation.left) > 1 or len(observation.right) > 1:
            self.fail('Multiple crates in a handoff slot')

    def fail(self, reason):
        self.fault = reason
        # Keep ownership after faults: automatic unlock would permit a collision.

    def acquire(self, owner, box, *, slot=0):
        if owner not in ('A', 'B', 'PUSHER'):
            raise ValueError(owner)
        if slot not in (0, 1):
            raise ValueError(slot)
        o = self.observation
        if self.owner is not None or self.fault or not o.valid or not o.arms_clear or not o.pusher_home:
            return None
        if owner == 'A':
            allowed = not (o.left if slot == 0 else o.right)
        elif owner == 'B':
            allowed = o.right == (box,) and box in o.stable
        else:
            allowed = o.left == (box,) and not o.right and box in o.stable
        if not allowed:
            return None
        self.sequence += 1
        self.owner, self.box = owner, box
        self.deposit_slot = slot if owner == 'A' else None
        self.action = f'{owner}:{box}:{self.sequence}'
        return self.action

    def commit(self, action):
        if action in self.completed:
            return False
        if action != self.action or self.owner is None or self.fault:
            raise RelayFault('Stale or faulted handoff commit')
        o = self.observation
        ready = o.valid and o.arms_clear and o.pusher_home
        if self.owner == 'A':
            occupied = o.left if self.deposit_slot == 0 else o.right
            ready = ready and occupied == (self.box,) and self.box in o.stable
        elif self.owner == 'B':
            ready = ready and not o.right
        else:
            ready = ready and not o.left and o.right == (self.box,) and self.box in o.stable
        if not ready:
            raise RelayFault('Physical handoff has not completed')
        if self.owner == 'PUSHER':
            self.transfers += 1
        self.completed.add(action)
        self.owner = self.box = self.action = None
        self.deposit_slot = None
        return True

    def status(self):
        o = self.observation
        return dict(owner=self.owner, action=self.action, in_transit=self.box if self.owner == 'PUSHER' else None,
                    left=list(o.left), right=list(o.right), fault=self.fault,
                    transfers=self.transfers, deposit_slot=self.deposit_slot)


class RelayInventory:
    def __init__(self, initial, count):
        if not 1 <= count <= len(initial):
            raise ValueError('Box count must be within the source inventory')
        self.queue = source_order(initial)[:count]
        self.owners = {i: 'SOURCE' for i in self.queue}
        self.completed = []
        self.target_reservations = {}

    def transfer(self, box, old, new):
        if self.owners.get(box) != old:
            raise RelayFault(f'Box {box}: expected owner {old}, found {self.owners.get(box)}')
        allowed = {'SOURCE': ('A',), 'A': ('L', 'R'), 'L': ('PUSHER',),
                   'PUSHER': ('R',), 'R': ('B',), 'B': ('TARGET',)}
        if new not in allowed.get(old, ()):
            raise RelayFault(f'Invalid ownership transition {old} -> {new}')
        self.owners[box] = new
        if new == 'TARGET':
            self.completed.append(box)

    def reserve_target(self, box):
        if self.owners.get(box) != 'R':
            raise RelayFault('Only a right-slot box may reserve a target')
        if box not in self.target_reservations:
            self.target_reservations[box] = len(self.target_reservations)
        return self.target_reservations[box]
