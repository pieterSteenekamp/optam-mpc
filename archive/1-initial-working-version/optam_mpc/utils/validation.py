"""Validation utilities for OptAM-MPC."""

from __future__ import annotations

from typing import Optional, Union

import numpy as np
from numpy.typing import ArrayLike, NDArray


# Type aliases for clarity
Array = NDArray[np.float64]
ScalarOrArray = Union[float, ArrayLike]


def vectorize(
    value: Optional[ScalarOrArray],
    length: int,
    name: str,
    default: Optional[float] = None,
) -> Array:
    """Convert input to a 1D float array of required length.

    Parameters
    ----------
    value : array-like or float, optional
        Input value to convert. If None, uses default if provided.
    length : int
        Required length of output array.
    name : str
        Name of the parameter (for error messages).
    default : float, optional
        Default value to use if value is None.

    Returns
    -------
    Array
        Copy of the input as a 1D float array with the required length.

    Raises
    ------
    ValueError
        If the input cannot be converted to the required shape,
        or if no default is provided when value is None.

    Examples
    --------
    >>> vectorize(1.0, 3, "test")
    array([1., 1., 1.])
    >>> vectorize([1.0, 2.0], 2, "test")
    array([1., 2.])
    """
    if value is None:
        if default is None:
            raise ValueError(f"{name} must be provided.")
        return np.full(length, default, dtype=float)

    # Handle scalar input (including 0D arrays)
    if np.isscalar(value) or (isinstance(value, np.ndarray) and value.ndim == 0):
        return np.full(length, float(value), dtype=float)

    # Convert to array and validate
    array = np.asarray(value, dtype=float)
    if array.ndim != 1:
        raise ValueError(f"{name} must be 1-dimensional, got {array.ndim}D.")
    if array.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {array.shape}.")
    if np.any(np.isnan(array)):
        raise ValueError(f"{name} must not contain NaN values.")

    return array.copy()


def require_finite(array: Array, name: str) -> None:
    """Check that all elements of an array are finite.

    Parameters
    ----------
    array : Array
        Array to check.
    name : str
        Name of the array (for error messages).

    Raises
    ------
    ValueError
        If any element is not finite (NaN or infinite).
    """
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values.")


def maximum_bound_violation(
    values: Array, lower: Array, upper: Array
) -> float:
    """Calculate the largest positive violation of element-wise bounds.

    Parameters
    ----------
    values : Array
        Values to check against bounds.
    lower : Array
        Lower bounds (can contain -inf for unbounded).
    upper : Array
        Upper bounds (can contain inf for unbounded).

    Returns
    -------
    float
        Largest violation. Returns 0.0 if all values are within bounds.
    """
    lower_violation = np.where(np.isfinite(lower), lower - values, 0.0)
    upper_violation = np.where(np.isfinite(upper), values - upper, 0.0)
    return float(max(0.0, np.max(lower_violation), np.max(upper_violation)))


def clip_to_bounds(values: Array, lower: Array, upper: Array) -> Array:
    """Clip values to element-wise bounds.

    Parameters
    ----------
    values : Array
        Values to clip.
    lower : Array
        Lower bounds (can contain -inf).
    upper : Array
        Upper bounds (can contain inf).

    Returns
    -------
    Array
        Clipped values.
    """
    return np.clip(values, lower, upper)
