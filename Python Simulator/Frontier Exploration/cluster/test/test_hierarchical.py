#
# This is part of "python-cluster". A library to group similar items together.
# Copyright (C) 2006    Michel Albert
#
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by the Free
# Software Foundation; either version 2.1 of the License, or (at your option)
# any later version.
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#

"""
Tests for hierarchical clustering.

.. note::

    Even though the results are lists, the order of items in the resulting
    clusters is non-deterministic. This should be taken into consideration when
    writing "expected" values!
"""

from difflib import SequenceMatcher
from math import sqrt
from sys import hexversion
import unittest

from cluster import HierarchicalClustering


class Py23TestCase(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(Py23TestCase, self).__init__(*args, **kwargs)
        if hexversion < 0x030000f0:
            self.assertCItemsEqual = self.assertItemsEqual
        else:
            self.assertCItemsEqual = self.assertCountEqual


class HClusterSmallListTestCase(Py23TestCase):
    """
    Test for Bug #1516204
    """

    def testClusterLen1(self):
        """
        Testing if hierarchical clustering a set of length 1 returns a set of
        length 1
        """
        cl = HierarchicalClustering([876], lambda x, y: abs(x - y))
        self.assertCItemsEqual([876], cl.getlevel(40))

    def testClusterLen0(self):
        """
        Testing if hierarchical clustering an empty list returns an empty list
        """
        cl = HierarchicalClustering([], lambda x, y: abs(x - y))
        self.assertEqual([], cl.getlevel(40))


class HClusterIntegerTestCase(Py23TestCase):

    def __init__(self, *args, **kwargs):
        super(HClusterIntegerTestCase, self).__init__(*args, **kwargs)
        if hexversion < 0x030000f0:
            self.assertCItemsEqual = self.assertItemsEqual
        else:
            self.assertCItemsEqual = self.assertCountEqual

    def setUp(self):
        self.__data = [791, 956, 676, 124, 564, 84, 24, 365, 594, 940, 398,
                       971, 131, 365, 542, 336, 518, 835, 134, 391]

    def testSingleLinkage(self):
        "Basic Hierarchical Clustering test with integers"
        cl = HierarchicalClustering(self.__data, lambda x, y: abs(x - y))
        result = cl.getlevel(40)

        # sort the values to make the tests less prone to algorithm changes
        result = [sorted(_) for _ in result]
        self.assertCItemsEqual([
            [24],
            [336, 365, 365, 391, 398],
            [518, 542, 564, 594],
            [676],
            [791],
            [835],
            [84, 124, 131, 134],
            [940, 956, 971],
        ], result)

    def testCompleteLinkage(self):
        "Basic Hierarchical Clustering test with integers"
        cl = HierarchicalClustering(self.__data,
                                    lambda x, y: abs(x - y),
                                    linkage='complete')
        result = cl.getlevel(40)

        # sort the values to make the tests less prone to algorithm changes
        result = sorted([sorted(_) for _ in result])

        expected = [
            [24],
            [84],
            [124, 131, 134],
            [336, 365, 365],
            [391, 398],
            [518],
            [542, 564],
            [594],
            [676],
            [791],
            [835],
            [940, 956, 971],
        ]
        self.assertEqual(result, expected)

    def testUCLUS(self):
        "Basic Hierarchical Clustering test with integers"
        cl = HierarchicalClustering(self.__data,
                                    lambda x, y: abs(x - y),
                                    linkage='uclus')
        expected = [
            [24],
            [84],
            [124, 131, 134],
            [336, 365, 365, 391, 398],
            [518, 542, 564],
            [594],
            [676],
            [791],
            [835],
            [940, 956, 971],
        ]
        result = sorted([sorted(_) for _ in cl.getlevel(40)])
        self.assertEqual(result, expected)

    def testAverageLinkage(self):
        cl = HierarchicalClustering(self.__data,
                                    lambda x, y: abs(x - y),
                                    linkage='average')
        # TODO: The current test-data does not really trigger a difference
        # between UCLUS and "average" linkage.
        expected = [
            [24],
            [84],
            [124, 131, 134],
            [336, 365, 365, 391, 398],
            [518, 542, 564],
            [594],
            [676],
            [791],
            [835],
            [940, 956, 971],
        ]
        result = sorted([sorted(_) for _ in cl.getlevel(40)])
        self.assertEqual(result, expected)

    def testUnmodifiedData(self):
        cl = HierarchicalClustering(self.__data, lambda x, y: abs(x - y))
        new_data = []
        [new_data.extend(_) for _ in cl.getlevel(40)]
        self.assertEqual(sorted(new_data), sorted(self.__data))

    def testMultiprocessing(self):
        cl = HierarchicalClustering(self.__data, lambda x, y: abs(x - y),
                                    num_processes=4)
        new_data = []
        [new_data.extend(_) for _ in cl.getlevel(40)]
        self.assertEqual(sorted(new_data), sorted(self.__data))


class HClusterStringTestCase(Py23TestCase):

    def sim(self, x, y):
        sm = SequenceMatcher(lambda x: x in ". -", x, y)
        return 1 - sm.ratio()

    def setUp(self):
        self.__data = ("Lorem ipsum dolor sit amet consectetuer adipiscing "
                       "elit Ut elit Phasellus consequat ultricies mi Sed "
                       "congue leo at neque Nullam").split()

    def testDataTypes(self):
        "Test for bug #?"
        cl = HierarchicalClustering(self.__data, self.sim)
        for item in cl.getlevel(0.5):
            self.assertEqual(
                type(item), type([]),
                "Every item should be a list!")

    def testCluster(self):
        "Basic Hierachical clustering test with strings"
        self.skipTest('These values lead to non-deterministic results. '
                      'This makes it untestable!')
        cl = HierarchicalClustering(self.__data, self.sim)
        self.assertEqual([
            ['ultricies'],
            ['Sed'],
            ['Phasellus'],
            ['mi'],
            ['Nullam'],
            ['sit', 'elit', 'elit', 'Ut', 'amet', 'at'],
            ['leo', 'Lorem', 'dolor'],
            ['congue', 'neque', 'consectetuer', 'consequat'],
            ['adipiscing'],
            ['ipsum'],
        ], cl.getlevel(0.5))

    def testUnmodifiedData(self):
        cl = HierarchicalClustering(self.__data, self.sim)
        new_data = []
        [new_data.extend(_) for _ in cl.getlevel(0.5)]
        self.assertEqual(sorted(new_data), sorted(self.__data))


class HClusterTuplesTestCase(Py23TestCase):
    '''
    Test case to cover the case where the data contains tuple-items

    See Github issue #20
    '''

    def testSingleLinkage(self):
        "Basic Hierarchical Clustering test with integers"

        def euclidian_distance(a, b):
            return sqrt(sum([pow(z[0] - z[1], 2) for z in zip(a, b)]))

        self.__data = [(1, 1), (1, 2), (1, 3)]
        cl = HierarchicalClustering(self.__data, euclidian_distance)
        result = cl.getlevel(40)
        self.assertIsNotNone(result)


if __name__ == '__main__':
    suite = unittest.TestSuite((
        unittest.makeSuite(HClusterIntegerTestCase),
        unittest.makeSuite(HClusterSmallListTestCase),
        unittest.makeSuite(HClusterStringTestCase),
    ))

    unittest.TextTestRunner(verbosity=2).run(suite)
