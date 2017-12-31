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


from cluster.util import ClusteringError, centroid, minkowski_distance


class KMeansClustering(object):
    """
    Implementation of the kmeans clustering method as explained in a tutorial_
    by *matteucc*.

    .. _tutorial: http://www.elet.polimi.it/upload/matteucc/Clustering/tutorial_html/kmeans.html

    Example:

      >>> from cluster import KMeansClustering
      >>> cl = KMeansClustering([(1,1), (2,1), (5,3), ...])
      >>> clusters = cl.getclusters(2)

    :param data: A list of tuples or integers.
    :param distance: A function determining the distance between two items.
        Default (if ``None`` is passed): It assumes the tuples contain numeric
        values and appiles a generalised form of the euclidian-distance
        algorithm on them.
    :param equality: A function to test equality of items. By default the
        standard python equality operator (``==``) is applied.
    :raises ValueError: if the list contains heterogeneous items or if the
        distance between items cannot be determined.
    """

    def __init__(self, data, distance=None, equality=None):
        self.__clusters = []
        self.__data = data
        self.distance = distance
        self.__initial_length = len(data)
        self.equality = equality

        # test if each item is of same dimensions
        if len(data) > 1 and isinstance(data[0], tuple):
            control_length = len(data[0])
            for item in data[1:]:
                if len(item) != control_length:
                    raise ValueError("Each item in the data list must have "
                                     "the same amount of dimensions. Item "
                                     "%r was out of line!" % (item,))
        # now check if we need and have a distance function
        if (len(data) > 1 and not isinstance(data[0], tuple) and
                distance is None):
            raise ValueError("You supplied non-standard items but no "
                             "distance function! We cannot continue!")
        # we now know that we have tuples, and assume therefore that it's
        # items are numeric
        elif distance is None:
            self.distance = minkowski_distance

    def getclusters(self, count):
        """
        Generates *count* clusters.

        :param count: The amount of clusters that should be generated.  count
            must be greater than ``1``.
        :raises ClusteringError: if *count* is out of bounds.
        """

        # only proceed if we got sensible input
        if count <= 1:
            raise ClusteringError("When clustering, you need to ask for at "
                                  "least two clusters! "
                                  "You asked for %d" % count)

        # return the data straight away if there is nothing to cluster
        if (self.__data == [] or len(self.__data) == 1 or
                count == self.__initial_length):
            return self.__data

        # It makes no sense to ask for more clusters than data-items available
        if count > self.__initial_length:
            raise ClusteringError(
                "Unable to generate more clusters than "
                "items available. You supplied %d items, and asked for "
                "%d clusters." % (self.__initial_length, count))

        self.initialise_clusters(self.__data, count)

        items_moved = True  # tells us if any item moved between the clusters,
                            # as we initialised the clusters, we assume that
                            # is the case

        while items_moved is True:
            items_moved = False
            for cluster in self.__clusters:
                for item in cluster:
                    res = self.assign_item(item, cluster)
                    if items_moved is False:
                        items_moved = res
        return self.__clusters

    def assign_item(self, item, origin):
        """
        Assigns an item from a given cluster to the closest located cluster.

        :param item: the item to be moved.
        :param origin: the originating cluster.
        """
        closest_cluster = origin
        for cluster in self.__clusters:
            if self.distance(item, centroid(cluster)) < self.distance(
                    item, centroid(closest_cluster)):
                closest_cluster = cluster

        if id(closest_cluster) != id(origin):
            self.move_item(item, origin, closest_cluster)
            return True
        else:
            return False

    def move_item(self, item, origin, destination):
        """
        Moves an item from one cluster to anoter cluster.

        :param item: the item to be moved.
        :param origin: the originating cluster.
        :param destination: the target cluster.
        """
        if self.equality:
            item_index = 0
            for i, element in enumerate(origin):
                if self.equality(element, item):
                    item_index = i
                    break
        else:
            item_index = origin.index(item)

        destination.append(origin.pop(item_index))

    def initialise_clusters(self, input_, clustercount):
        """
        Initialises the clusters by distributing the items from the data.
        evenly across n clusters

        :param input_: the data set (a list of tuples).
        :param clustercount: the amount of clusters (n).
        """
        # initialise the clusters with empty lists
        self.__clusters = []
        for _ in range(clustercount):
            self.__clusters.append([])

        # distribute the items into the clusters
        count = 0
        for item in input_:
            self.__clusters[count % clustercount].append(item)
            count += 1
