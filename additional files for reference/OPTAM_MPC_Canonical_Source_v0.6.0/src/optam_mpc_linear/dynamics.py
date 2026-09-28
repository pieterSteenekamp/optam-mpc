"""Linear path states with exact ZOH lag discretisation and sampled dead time.

Works with Python numbers or CasADi expressions. No process-specific equations.
"""
import math


class PathBank:
    def __init__(self, paths, sample_seconds, source_initial, output_ids):
        self.paths = paths
        self.dt = sample_seconds
        self.source_initial = source_initial
        self.output_ids = tuple(output_ids)
        self.layout = []
        cursor = 0
        for path in paths:
            count = int(round(path.dead_time_seconds / sample_seconds))
            z_index = cursor if path.type != "integrator" else None
            cursor += int(z_index is not None)
            self.layout.append((z_index, cursor, count))
            cursor += count
        self.dimension = cursor
        self.initial = (0.0,) * cursor

    def advance(self, state, sources):
        result = list(state)
        increments = {key: 0.0 for key in self.output_ids}
        rates = dict(increments)
        for path, (z, start, count) in zip(self.paths, self.layout):
            value = sources[path.source] - self.source_initial[path.source]
            delayed = state[start+count-1] if count else value
            if count:
                result[start:start+count] = [value, *state[start:start+count-1]]
            if path.type == "integrator":
                rates[path.destination] += path.integrating_gain * delayed
            else:
                a = math.exp(-self.dt/path.time_constant_seconds) if path.type == "first_order" else 0.0
                next_z = a * state[z] + (1-a) * path.gain * delayed
                result[z] = next_z
                increments[path.destination] += next_z - state[z]
        for key in increments:
            increments[key] += self.dt/60 * rates[key]
        return tuple(result), increments


def model_check_value(definition, check):
    """Independent continuous-time step formula, not the path-state recurrence."""
    paths = [p for p in definition.paths if p.source == check["source"] and p.destination == check["destination"]]
    metric = check.get("metric", "rate")
    total = 0.0
    for path in paths:
        if metric == "rate":
            if path.type != "integrator" or path.dead_time_seconds != 0:
                raise ValueError("Rate checks require zero-delay integrator paths; use step_response otherwise.")
            total += path.integrating_gain * check["step"]
        elif metric == "steady_change":
            if path.type == "integrator":
                raise ValueError("An integrator has no finite steady-state step change.")
            total += path.gain * check["step"]
        else:
            elapsed = max(0.0, check["time_seconds"] - path.dead_time_seconds)
            if path.type == "integrator":
                total += path.integrating_gain * check["step"] * elapsed/60
            elif path.type == "first_order":
                total += path.gain * check["step"] * (1-math.exp(-elapsed/path.time_constant_seconds))
            elif elapsed > 0:
                total += path.gain * check["step"]
    return total
