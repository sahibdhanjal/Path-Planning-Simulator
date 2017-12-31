#
# This is part of "python-cluster". A library to group similar items together.
# Copyright (C) 2006    Michel Albert
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
# for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#


class BaseClusterMethod(object):
    """
    The base class of all clustering methods.

    :param input: a list of objects
    :distance_function: a function returning the distance - or opposite of
        similarity ``(distance = -similarity)`` - of two items from the input.
        In other words, the closer the two items are related, the smaller this
        value needs to be.  With 0 meaning they are exactly the same.

    .. note::
        The distance function should always return the absolute distance between
        two given items of the list. Say::

            distance(input[1], input[4]) = distance(input[4], input[1])

        This is very important for the clustering algorithm to work!  Naturally,
        the data returned by the distance function MUST be a comparable
        datatype, so you can perform arithmetic comparisons on them (``<`` or
        ``>``)! The simplest examples would be floats or ints. But as long as
        they are comparable, it's ok.
    """

    def __init__(self, input, distance_function, progress_callback=None):
        self.distance = distance_function
        self._input = input    # the original input
        self._data = input[:]  # clone the input so we can work with it
                               # without distroying the original data.
        self.progress_callback = progress_callback

    def topo(self):
        """
        Returns the structure (topology) of the cluster.

        See :py:meth:`~cluster.cluster.Cluster.topology` for more information.
        """
        return self.data[0].topology()

    @property
    def data(self):
        """
        Returns the data that is currently in process.
        """
        return self._data

    @property
    def raw_data(self):
        """
        Returns the raw data (data without being clustered).
        """
        return self._input
