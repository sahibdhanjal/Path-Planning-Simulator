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

import unittest

from cluster import KMeansClustering
try:
    import numpy
    NUMPY_AVAILABLE = True
except:
    NUMPY_AVAILABLE = False


@unittest.skipUnless(NUMPY_AVAILABLE,
                     'numpy not available. Associated test will not be loaded!')
class NumpyTests(unittest.TestCase):

    def testNumpyRandom(self):
        data = numpy.random.rand(500, 2)
        cl = KMeansClustering(data, lambda p0, p1: (
            p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2, numpy.array_equal)
        cl.getclusters(10)
