import time
import enum
import numpy as np


class EulerState(enum.Enum):
  NORMAL = 1
  POS_SINGULAR = 2
  NEG_SINGULAR = 3


ZERO_TOLERANCE = 1e-6


def normalize(k):
  k = k / np.linalg.norm(k)
  return k


def inner_prod(a, b):
    return (a * b).sum(axis=-1)


def circle_distance(a, b, rad_unit=True):
  unit = 2 * np.pi if rad_unit else 360
  d = np.absolute((b - a)) % unit
  cd = np.absolute(d - unit)
  return np.minimum(d, cd)


def infer_angle(left, right, reference):
  cosine = inner_prod(left, right)
  lesser_angle = np.arctan2(
    np.sqrt(1 - cosine * cosine), cosine)
  if np.abs(lesser_angle) < ZERO_TOLERANCE:
    angle = 0
  # atan2 is not continuous around (-1, 0)
  elif np.abs(lesser_angle - np.pi) < ZERO_TOLERANCE:
    angle = np.pi
  else:
    ori = np.cross(left, right)
    sign = np.sign(inner_prod(ori, reference))
    angle = lesser_angle * sign
  return angle


def extract_position(hm):
  return hm[:3, 3]


def infer_wrist_center(eef, eef_rel):
  return np.matmul(eef, np.linalg.inv(eef_rel))


def infer_waist_state(wc_pos):
  return np.arctan2(wc_pos[1], wc_pos[0])


def solve_triangle(left, right, opposite):
  ret = (left ** 2 + right ** 2
         - opposite ** 2) / (2 * left * right)
  return np.arccos(ret)


def solution_dict(waist, shoulder, elbow, wrist):

  ret = {'joint_1': waist, 'joint_2': shoulder,
         'joint_3': elbow}

  for i in range(3):
    ret['joint_{}'.format(i + 4)] = wrist[i]
  return ret


def select_nearest(current_state, solutions):
  distances = []
  for solution in solutions:
    distance = sum(circle_distance(
      solution[x], current_state[x]) for x in current_state)
    distances.append(distance)

  index = np.argmin(distances)
  print 'distance of two solutions:', \
    distances, 'selected:', index
  return solutions[index]


def hm_distance(hm1, hm2):
  pos1 = extract_position(hm1)
  pos2 = extract_position(hm2)
  return np.linalg.norm(pos1 - pos2)


def solve_zyz_euler(rotation):
  q2 = np.arccos(rotation[2, 2])
  # singular case
  if abs(q2) < ZERO_TOLERANCE:
    # set q3 to zero
    q3, q1 = 0, np.arctan2(
      rotation[1, 0], rotation[0, 0])
    return (q1, q2, q3), EulerState.POS_SINGULAR
  elif abs(q2 - np.pi) < ZERO_TOLERANCE:
    q3, q1 = 0, np.arctan2(
      -rotation[1, 0], -rotation[0, 0])
    return (q1, q2, q3), EulerState.NEG_SINGULAR
  else:
    sq2 = np.sign(np.sin(q2))
    q1 = np.arctan2(sq2 * rotation[1, 2],
                    sq2 * rotation[0, 2])
    q3 = np.arctan2(sq2 * rotation[2, 1],
                    -sq2 * rotation[2, 0])
  return (q1, q2, q3), EulerState.NORMAL


class KukaGeometricIK(object):

  def __init__(self, dag):
    self._graph = dag[0].graph
    dh_frames = self._graph.dh_frames
    self._upper_arm = hm_distance(
      dh_frames[3], dh_frames[2])
    self._lower_arm = hm_distance(
      dh_frames[4], dh_frames[3])
    # shoulder position at zero pos
    self._shoulder_pos = extract_position(dh_frames[2])
    links = self._graph.dh_links
    self._shoulder = links[1]
    self._elbow = links[2]
    self._wrists = links[3:]
    eef_link = self._graph.fixed_links[-1]
    self._eef_rel = eef_link.rel

  def infer_joints(
      self, current_state, eef):
    now = time.time()
    wc = infer_wrist_center(eef, self._eef_rel)
    wc_pos = extract_position(wc)
    waist_state = infer_waist_state(wc_pos)
    shoulder_states = self.infer_shoulder(
      wc_pos, waist_state)
    solutions = []
    for ss in shoulder_states:
      elbow_state = self.infer_elbow(
        waist_state, ss, wc_pos)
      # unreachable
      if elbow_state is None:
        continue
      wrist_states = self.infer_wrist(
        waist_state, ss, elbow_state, wc)
      # unreachable
      if wrist_states is None:
        continue
      solution = solution_dict(
        waist_state, ss, elbow_state, wrist_states)
      solutions.append(solution)

    if len(solutions) == 0:
      # unreachable
      print 'unreachable eef state'
      return None

    ret = self._graph.forward(solutions[0], return_dh=False)
    err = np.sum(np.round(np.abs(ret[-1][1] - eef), 4))
    print 'eef validation', err
    duration = time.time() - now
    print 'time usage:', duration

    if len(solutions) == 1:
      return solutions[0]
    return select_nearest(current_state, solutions)

  def infer_shoulder(self, wc_pos, waist):
    forward = self._graph.forward(
      {'joint_1': waist}, partial_index=1)
    waist_rot = forward[1][1][:3, :3].transpose()
    wc_pos = np.matmul(waist_rot, wc_pos)
    wc_shoulder = wc_pos - self._shoulder_pos
    wc_norm = np.linalg.norm(wc_shoulder)
    angle = solve_triangle(
      self._upper_arm, wc_norm, self._lower_arm)

    wc_angle = np.arctan2(wc_shoulder[2], wc_shoulder[0])
    solu1 = np.pi / 2 - (wc_angle + angle)
    solu2 = np.pi / 2 - (wc_angle - angle)
    ret = (solu1, solu2)
    ret = filter(self._shoulder.reachable, ret)
    return ret

  def infer_elbow(self, waist, shoulder, wc_pos):
    forward = self._graph.forward(
      {'joint_1': waist,
       'joint_2': shoulder,
       'joint_3': 0, 'joint_4': 0},
      partial_index=4)
    elbow_hm = forward[3][1]
    elbow_axis = normalize(elbow_hm[:3, 2])
    elbow_pos = extract_position(elbow_hm)
    wc_zero = extract_position(forward[4][1])
    wc_elbow_zero = normalize(wc_zero - elbow_pos)
    wc_elbow = normalize(wc_pos - elbow_pos)
    angle = infer_angle(
      wc_elbow_zero, wc_elbow, elbow_axis)
    if not self._elbow.reachable(angle):
      return None
    else:
      return angle

  def infer_wrist(
      self, waist, shoulder, elbow, wc_hm):

    forward = self._graph.forward(
      {'joint_1': waist, 'joint_2': shoulder,
       'joint_3': elbow, 'joint_4': 0,
       'joint_5': 0, 'joint_6': 0})

    assert np.allclose(
      extract_position(wc_hm),
      extract_position(forward[-1][1]))
    zero_ori = forward[-1][1][:3, :3]
    wc_ori = wc_hm[:3, :3]
    delta = np.matmul(zero_ori.transpose(), wc_ori)
    qs, is_singular = solve_zyz_euler(delta)

    if is_singular != EulerState.NORMAL:
      # try additional step to divided rotation on q1 and q3
      factor = 1 if is_singular == EulerState.POS_SINGULAR else -1
      first_range = self._wrists[0].reachable_range
      second_range = self._wrists[2].reachable_range
      second_range = (factor * second_range[0],
                      factor * second_range[1])
      upper = first_range[1] + max(second_range)
      lower = first_range[0] + min(second_range)
      if not lower <= qs[0] <= upper:
        return None

      if lower <= qs[0] <= first_range[0]:
        q1 = first_range[0]
        q2 = factor * (qs[0] - first_range[0])

      elif first_range[0] <= qs[0] <= first_range[1]:
        q1, q2 = qs[0], 0
      else:
        q1 = first_range[1]
        q2 = factor * (qs[0] - first_range[1])
      return q1, 0, q2
    assert len(qs) == len(self._wrists)
    for q, l in zip(qs, self._wrists):
      if not l.reachable(q):
        return None
    return tuple(qs)
