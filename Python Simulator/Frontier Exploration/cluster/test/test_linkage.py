import unittest

from cluster.linkage import single, complete, uclus, average


class LinkageMethods(unittest.TestCase):

    def setUp(self):
        self.set_a = [1, 2, 3, 4]
        self.set_b = [10, 11, 12, 13, 14, 15, 100]
        self.dist = lambda x, y: abs(x-y)  # NOQA

    def test_single_distance(self):
        result = single(self.set_a, self.set_b, self.dist)
        expected = 6
        self.assertEqual(result, expected)

    def test_complete_distance(self):
        result = complete(self.set_a, self.set_b, self.dist)
        expected = 99
        self.assertEqual(result, expected)

    def test_uclus_distance(self):
        result = uclus(self.set_a, self.set_b, self.dist)
        expected = 10.5
        self.assertEqual(result, expected)

    def test_average_distance(self):
        result = average(self.set_a, self.set_b, self.dist)
        expected = 22.5
        self.assertEqual(result, expected)
