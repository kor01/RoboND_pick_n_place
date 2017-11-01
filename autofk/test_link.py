import unittest
import sympy as sp
from autofk.link import homogenous_transform


class TestDHToHomogenous(unittest.TestCase):

  def test_construct(self):

    theta1, theta2, theta3, \
      theta4, theta5, theta6 = sp.symbols('theta1:7')

    dh_table = [[0, 0, 0, theta1],
                [0.35, -sp.pi/2, 0, -sp.pi/2 + theta2],
                [1.25, 0, 0, theta3],
                [0.056, sp.pi/2, 1.5, theta4],
                [0, -sp.pi/2, 0, theta5],
                [0, sp.pi/2, 0, theta6]]

    matrices = [homogenous_transform(*x) for x in dh_table]

    print matrices
