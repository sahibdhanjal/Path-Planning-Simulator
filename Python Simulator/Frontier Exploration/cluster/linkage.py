from __future__ import division
from functools import wraps


def cached(fun):
    """
    memoizing decorator for linkage functions.

    Parameters have been hardcoded (no ``*args``, ``**kwargs`` magic), because,
    the way this is coded (interchangingly using sets and frozensets) is true
    for this specific case. For other cases that is not necessarily guaranteed.
    """

    _cache = {}

    @wraps(fun)
    def newfun(a, b, distance_function):
        frozen_a = frozenset(a)
        frozen_b = frozenset(b)
        if (frozen_a, frozen_b) not in _cache:
            result = fun(a, b, distance_function)
            _cache[(frozen_a, frozen_b)] = result
        return _cache[(frozen_a, frozen_b)]
    return newfun


@cached
def single(a, b, distance_function):
    """
    Given two collections ``a`` and ``b``, this will return the distance of the
    points which are closest together.  ``distance_function`` is used to
    determine the distance between two elements.

    Example::

        >>> single([1, 2], [3, 4], lambda x, y: abs(x-y))
        1  # (distance between 2 and 3)
    """
    left_a, right_a = min(a), max(a)
    left_b, right_b = min(b), max(b)
    result = min(distance_function(left_a, right_b),
                 distance_function(left_b, right_a))
    return result


@cached
def complete(a, b, distance_function):
    """
    Given two collections ``a`` and ``b``, this will return the distance of the
    points which are farthest apart.  ``distance_function`` is used to determine
    the distance between two elements.

    Example::

        >>> single([1, 2], [3, 4], lambda x, y: abs(x-y))
        3  # (distance between 1 and 4)
    """
    left_a, right_a = min(a), max(a)
    left_b, right_b = min(b), max(b)
    result = max(distance_function(left_a, right_b),
                 distance_function(left_b, right_a))
    return result


@cached
def average(a, b, distance_function):
    """
    Given two collections ``a`` and ``b``, this will return the mean of all
    distances. ``distance_function`` is used to determine the distance between
    two elements.

    Example::

        >>> single([1, 2], [3, 100], lambda x, y: abs(x-y))
        26
    """
    distances = [distance_function(x, y)
                 for x in a for y in b]
    return sum(distances) / len(distances)


@cached
def uclus(a, b, distance_function):
    """
    Given two collections ``a`` and ``b``, this will return the *median* of all
    distances. ``distance_function`` is used to determine the distance between
    two elements.

    Example::

        >>> single([1, 2], [3, 100], lambda x, y: abs(x-y))
        2.5
    """
    distances = sorted([distance_function(x, y)
                        for x in a for y in b])
    midpoint, rest = len(distances) // 2, len(distances) % 2
    if not rest:
        return sum(distances[midpoint-1:midpoint+1]) / 2
    else:
        return distances[midpoint]
