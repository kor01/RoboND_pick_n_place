import xacro
import numpy as np
from definitions import Link
from definitions import Joint
from definitions import JointLimits


def parse_vector(vec):
  ret = [float(x) for x in vec.split(' ')]
  ret = np.array(ret, dtype=np.float64)
  ret.flags.writeable = False
  return ret


def parse_sub_attribute(
    dom, elem_name, attr_name,
    parser=parse_vector):
  attr = dom.getElementsByTagName(
    elem_name)[0].getAttribute(attr_name)
  ret = parser(attr)
  return ret


def symmetric_assignment(matrix, value, i, j):
  matrix[i, j] = value
  matrix[j, i] = value


def parse_inertia(dom):
  dom = dom.getElementsByTagName('inertia')[0]
  ret = np.zeros((3, 3), dtype=np.float64)
  axis = ('x', 'y', 'z')
  for i in range(len(axis)):
    for j in range(i, len(axis)):
      name = 'i' + axis[i] + axis[j]
      attr = float(dom.getAttribute(name))
      symmetric_assignment(ret, attr, i, j)
  return ret


# noinspection PyTypeChecker
def parse_links(links):
  ret = {}
  for l in links:
    name = l.getAttribute('name')
    inertial = l.getElementsByTagName('inertial')
    # default value
    if len(inertial) == 0:
      position = np.zeros((3,), dtype=np.float64)
      orientation = np.zeros((3,), dtype=np.float64)
      mass = None
      inertia_matrix = None
    else:
      inertial = inertial[0]
      position = parse_sub_attribute(inertial, 'origin', 'xyz')
      orientation = parse_sub_attribute(inertial, 'origin', 'rpy')
      mass = parse_sub_attribute(inertial, 'mass', 'value', float)
      inertia_matrix = parse_inertia(inertial)
      inertia_matrix.flags.writeable = False

    link = Link(name=name, position=position,
                orientation=orientation,
                mass=mass, inertia=inertia_matrix)
    ret[link.name] = link
  return ret


def parse_joint_limit(dom):
  if len(dom) == 0:
    return None

  dom = dom[0]
  lower = float(dom.getAttribute('lower'))
  upper = float(dom.getAttribute('upper'))
  effort = float(dom.getAttribute('effort'))
  velocity = float(dom.getAttribute('velocity'))
  return JointLimits(
    lower=lower, upper=upper,
    effort=effort, velocity=velocity)


# noinspection PyTypeChecker
def parse_joints(joints, links):

  ret = []
  for j in joints:
    name = j.getAttribute('name')
    jtype = j.getAttribute('type')
    orientation = parse_sub_attribute(j, 'origin', 'rpy')
    position = parse_sub_attribute(j, 'origin', 'xyz')
    parent = links[parse_sub_attribute(
      j, 'parent', 'link', parser=str)]
    child = links[parse_sub_attribute(
      j, 'child', 'link', parser=str)]
    if jtype != 'fixed':
      axis = parse_sub_attribute(j, 'axis', 'xyz')
      limits = parse_joint_limit(
        j.getElementsByTagName('limit'))
    else:
      limits, axis = None, None
    joint = Joint(name=name, position=position,
                  orientation=orientation,
                  axis=axis, type=jtype, parent=parent,
                  child=child, limits=limits)
    ret.append(joint)
  return tuple(ret)


def get_links_and_joints(dom):
  robot = dom.getElementsByTagName('robot')
  assert len(robot) == 1, len(robot)
  robot = robot[0]
  joints, links = [], []
  for node in robot.childNodes:
    if node.nodeName == 'joint':
      joints.append(node)
    elif node.nodeName == 'link':
      links.append(node)
  return joints, links


class URDFParser(object):
  def __init__(self, path=None, data=None):
    assert data or path
    self._dom = xacro.parse(data, path)
    joints, links = get_links_and_joints(self._dom)
    self._links = parse_links(links)
    self._joints = tuple(parse_joints(joints, self._links))

  @property
  def links(self):
    return self._links

  @property
  def joints(self):
    return self._joints


