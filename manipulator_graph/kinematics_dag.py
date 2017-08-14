from collections import defaultdict
from serial_subgraph import SerialSubGraph


def joint_topology(joints):
  link_child, link_parent = defaultdict(list), {}
  for j in joints:
    link_child[j.parent.name].append(j.name)
    link_parent[j.child.name] = j.name
  joint_parents, joint_children = {}, {}
  for j in joints:
    key = j.parent.name
    if key in link_parent:
      joint_parents[j.name] = link_parent[key]
    else:
      joint_parents[j.name] = None
    key = j.child.name
    if key in link_child:
      joint_children[j.name] = tuple(link_child[key])
    else:
      joint_children[j.name] = tuple()
  return joint_parents, joint_children


class GraphNode(object):

  def __init__(self, graph,
               parent, children):

    assert isinstance(graph, SerialSubGraph)
    self._graph = graph
    self._parent = parent
    self._children = children

  @property
  def graph(self):
    return self._graph

  @property
  def parent(self):
    return self._parent

  @property
  def children(self):
    return self._children


def forward_expand(j, joint_children):
  ret, j = [], [j]
  while len(j) == 1:
    ret.append(j[0])
    j = joint_children[j[0]]
  return tuple(ret)


def double_expand(joint, joint_parent, joint_children):
  ret = []
  j = joint
  while j is not None:
    ret.insert(0, j)
    j = joint_parent[j]

  j = joint_children[joint]
  while len(j) == 1:
    ret.append(j[0])
    j = joint_children[j[0]]
  return tuple(ret)


def partition_as_serial_dag(joints):

  joint_parent, joint_children = joint_topology(joints)
  joints = map(lambda x: x.name, joints)
  # find branches
  branches = [v for _, v in
              joint_children.items() if len(v) > 1]

  serials = []
  for v in branches:
    for j in v:
      serials.append(forward_expand(j, joint_children))

  nonroot_joints = set(sum(serials, tuple()))
  root_joints = filter(lambda x: x not in nonroot_joints, joints)

  root = double_expand(
    root_joints[0], joint_parent, joint_children)
  assert len(root_joints) == len(root)
  serials.insert(0, root)

  assert len(sum(serials, tuple())) == len(joints), 'not connected'
  serials = tuple(serials)

  serial_ends = {s[-1]: i for i, s in enumerate(serials)}
  serial_parent = [None for _ in serials]
  for i, s in enumerate(serials):
    key = joint_parent[s[0]]
    if key in serial_ends:
      serial_parent[i] = serial_ends[key]
    else:
      serial_parent[i] = None

  serial_children = [tuple() for _ in serials]
  for c, p in enumerate(serial_parent):
    if p is None:
      continue
    assert isinstance(p, int)
    serial_children[p] += (c,)

  return serials, tuple(serial_parent), \
         tuple(serial_children)


class KinematicsDag(object):

  def __init__(self, joints):
    serials, children, parents \
      = partition_as_serial_dag(joints)
    joint_map = {x.name: x for x in joints}
    dag = []
    for i, s in enumerate(serials):
      serial = map(lambda x: joint_map[x], s)
      graph = SerialSubGraph(serial)
      parent, chil = parents[i], children[i]
      node = GraphNode(graph, parent, chil)
      dag.append(node)
    self._dag = tuple(dag)

  def __len__(self):
    return len(self._dag)

  @property
  def first(self):
    return self._dag[0]

  def __getitem__(self, i):
    return self._dag[i]

