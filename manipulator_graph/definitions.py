from collections import namedtuple


JointLimits = namedtuple(
  'JointLimits', ('effort', 'lower', 'upper', 'velocity'))

Joint = namedtuple(
  'Joint', ('name', 'position', 'orientation',
            'axis', 'type', 'parent', 'child', 'limits'))


Link = namedtuple('Link', ('name', 'position',
                           'orientation', 'mass', 'inertia'))

DHArgument = namedtuple(
  'DHArgument', ('a', 'alpha', 'd', 'theta', 'initial'))

# used to group link and joints
NameScope = namedtuple('NameScope', ('name', 'predicate'))


def string_match_scope(name, character):
  return NameScope(
    name=name, predicate=lambda x: character in x)
