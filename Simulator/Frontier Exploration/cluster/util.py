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

from __future__ import print_function
import logging


logger = logging.getLogger(__name__)


class ClusteringError(Exception):
    pass


def flatten(L):
    """
    Flattens a list.

    Example:

    >>> flatten([a,b,[c,d,[e,f]]])
    [a,b,c,d,e,f]
    """
    if not isinstance(L, list):
        return [L]

    if L == []:
        return L

    return flatten(L[0]) + flatten(L[1:])


def fullyflatten(container):
    """
    Completely flattens out a cluster and returns a one-dimensional set
    containing the cluster's items. This is useful in cases where some items of
    the cluster are clusters in their own right and you only want the items.

    :param container: the container to flatten.
    """
    flattened_items = []

    for item in container:
        if hasattr(item, 'items'):
            flattened_items = flattened_items + fullyflatten(item.items)
        else:
            flattened_items.append(item)

    return flattened_items


def median(numbers):
    """
    Return the median of the list of numbers.
    see: http://mail.python.org/pipermail/python-list/2004-December/294990.html
    """

    # Sort the list and take the middle element.
    n = len(numbers)
    copy = sorted(numbers)
    if n & 1:  # There is an odd number of elements
        return copy[n // 2]
    else:
        return (copy[n // 2 - 1] + copy[n // 2]) / 2.0


def mean(numbers):
    """
    Returns the arithmetic mean of a numeric list.
    see: http://mail.python.org/pipermail/python-list/2004-December/294990.html
    """
    return float(sum(numbers)) / float(len(numbers))


def minkowski_distance(x, y, p=2):
    """
    Calculates the minkowski distance between two points.

    :param x: the first point
    :param y: the second point
    :param p: the order of the minkowski algorithm. If *p=1* it is equal
        to the manhatten distance, if *p=2* it is equal to the euclidian
        distance. The higher the order, the closer it converges to the
        Chebyshev distance, which has *p=infinity*.
    """
    from math import pow
    assert len(y) == len(x)
    assert len(x) >= 1
    sum = 0
    for i in range(len(x)):
        sum += abs(x[i] - y[i]) ** p
    return pow(sum, 1.0 / float(p))


def magnitude(a):
    "calculates the magnitude of a vecor"
    from math import sqrt
    sum = 0
    for coord in a:
        sum += coord ** 2
    return sqrt(sum)


def dotproduct(a, b):
    "Calculates the dotproduct between two vecors"
    assert(len(a) == len(b))
    out = 0
    for i in range(len(a)):
        out += a[i] * b[i]
    return out


def centroid(data, method=median):
    "returns the central vector of a list of vectors"
    out = []
    for i in range(len(data[0])):
        out.append(method([x[i] for x in data]))
    return tuple(out)
