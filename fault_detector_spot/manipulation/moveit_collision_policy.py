"""Prepare MoveIt occupancy collision rules without changing other rules."""

from copy import deepcopy

from moveit_msgs.msg import AllowedCollisionEntry


# moveit::planning_scene::PlanningScene::OCTOMAP_NS in MoveIt 2.5.
OCTOMAP_OBJECT_ID = "<octomap>"


def occupancy_collision_matrix(matrix, *, ignore_environment_collisions):
    """Return an ACM with only occupancy-related effective rules changed.

    Default-only names need explicit occupancy pairs: MoveIt combines two
    defaults with AND, so an object's false default would defeat bypass.
    Expanding those names preserves their existing effective pair rules.
    """
    if type(ignore_environment_collisions) is not bool:
        raise TypeError("Ignore environmental collisions must be a boolean")
    names = list(matrix.entry_names)
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    if (
        len(set(names)) != len(names)
        or len(defaults) != len(matrix.default_entry_names)
        or len(matrix.default_entry_names) != len(matrix.default_entry_values)
        or len(matrix.entry_values) != len(names)
        or any(len(row.enabled) != len(names) for row in matrix.entry_values)
    ):
        raise ValueError("MoveIt collision matrix has invalid dimensions or names")
    for i, row in enumerate(matrix.entry_values):
        for j in range(i):
            if row.enabled[j] != matrix.entry_values[j].enabled[i]:
                raise ValueError("MoveIt collision matrix must be symmetric")

    indices = {name: index for index, name in enumerate(names)}
    expanded = list(dict.fromkeys(names + list(defaults) + [OCTOMAP_OBJECT_ID]))

    def allowed(first, second):
        if OCTOMAP_OBJECT_ID in (first, second):
            return ignore_environment_collisions
        if first in indices and second in indices:
            return matrix.entry_values[indices[first]].enabled[indices[second]]
        values = [defaults[name] for name in (first, second) if name in defaults]
        return bool(values) and all(values)

    result = deepcopy(matrix)
    result.entry_names = expanded
    result.entry_values = [
        AllowedCollisionEntry(enabled=[allowed(first, second) for second in expanded])
        for first in expanded
    ]
    defaults[OCTOMAP_OBJECT_ID] = ignore_environment_collisions
    result.default_entry_names = list(defaults)
    result.default_entry_values = list(defaults.values())
    return result
