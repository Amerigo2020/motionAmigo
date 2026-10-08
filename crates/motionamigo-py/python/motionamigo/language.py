"""From language to motion: resolve the target object of an instruction, then plan the pick.

The resolution step is pluggable through the :class:`Resolver` protocol. By default the
spatialAmigo package is used when it is importable, otherwise a small built-in resolver that
understands object ids, labels and the relations left, right, front, behind and near.
"""

from __future__ import annotations

import json
import math
import re
from os import PathLike, fspath
from typing import Any, NamedTuple, Optional, Protocol, Union

from ._motionamigo import PickResult, Robot, plan_pick

SceneLike = Union[str, PathLike, dict]

# Longer phrases first so that "to the left of" wins over "left".
_RELATIONS = {
    "left": ("left",),
    "right": ("right",),
    "front": ("in front", "front"),
    "behind": ("behind", "back of", "in back"),
    "near": ("near", "next to", "closest to", "nearest", "beside"),
}


class ResolutionError(ValueError):
    """The instruction does not refer to exactly one object of the scene."""


class Resolver(Protocol):
    """Maps an instruction and a scene (as a dict in the v0.1 format) to an object id."""

    def __call__(self, scene: dict, instruction: str) -> str: ...


class InstructionResult(NamedTuple):
    object_id: str
    pick: PickResult


def load_scene(scene: SceneLike) -> dict:
    """Returns the scene as a dict; paths are read as JSON."""
    if isinstance(scene, dict):
        return scene
    with open(fspath(scene), encoding="utf-8") as f:
        return json.load(f)


def builtin_resolver(scene: dict, instruction: str) -> str:
    """Resolves ids, labels and one relation ("the mug left of the laptop") in the scene.

    Left, right, front and behind are seen from the scene's viewpoint (default: looking along +x),
    so "left" means larger y for the default view.
    """
    text = " " + re.sub(r"[^a-z0-9_ ]+", " ", instruction.lower().replace("-", " ")) + " "
    objects = scene["objects"]
    by_id = [o for o in objects if f" {o['id'].lower()} " in text]
    if len(by_id) == 1:
        return by_id[0]["id"]

    # Labels in order of appearance: the first one is the target, the second one the anchor.
    found = []
    for label in {o["label"] for o in objects}:
        m = re.search(rf" {re.escape(label.lower())}s? ", text)
        if m:
            found.append((m.start(), label))
    found.sort()
    if not found:
        raise ResolutionError(f"no object of the scene is mentioned in {instruction!r}")
    target = found[0][1]
    candidates = [o for o in objects if o["label"] == target]

    relation = None
    for name, phrases in _RELATIONS.items():
        if any(f" {p} " in text for p in phrases):
            relation = name
            break
    if relation is None or len(found) < 2:
        if len(candidates) > 1:
            ids = ", ".join(o["id"] for o in candidates)
            raise ResolutionError(f"{instruction!r} is ambiguous: {ids}")
        return candidates[0]["id"]

    anchors = [o for o in objects if o["label"] == found[1][1]]
    if len(anchors) != 1:
        raise ResolutionError(f"the anchor {found[1][1]!r} in {instruction!r} is not unique")
    anchor = anchors[0]["center"]
    fx, fy = 1.0, 0.0
    if "viewpoint" in scene:
        vp = scene["viewpoint"]
        fx, fy = vp["look_at"][0] - vp["position"][0], vp["look_at"][1] - vp["position"][1]
        norm = math.hypot(fx, fy) or 1.0
        fx, fy = fx / norm, fy / norm

    def offset(o: dict[str, Any]) -> tuple[float, float]:
        dx, dy = o["center"][0] - anchor[0], o["center"][1] - anchor[1]
        return dx * fx + dy * fy, -dx * fy + dy * fx  # (forward, left) seen from the viewpoint

    if relation == "near":
        dist = sorted((math.hypot(*offset(o)), o["id"]) for o in candidates)
        if len(dist) > 1 and math.isclose(dist[0][0], dist[1][0], abs_tol=1e-6):
            raise ResolutionError(f"{instruction!r} is ambiguous: {dist[0][1]}, {dist[1][1]}")
        return dist[0][1]
    # ponytail: half-plane test, a soft sector score (as in spatialAmigo) if scenes get cluttered.
    test = {
        "left": lambda f, s: s > 0,
        "right": lambda f, s: s < 0,
        "front": lambda f, s: f < 0,  # between the anchor and the viewer
        "behind": lambda f, s: f > 0,
    }[relation]
    hits = [o["id"] for o in candidates if test(*offset(o))]
    if len(hits) != 1:
        what = "ambiguous: " + ", ".join(hits) if hits else "not satisfied by any object"
        raise ResolutionError(f"{instruction!r} is {what}")
    return hits[0]


def spatialamigo_resolver(min_margin: float = 0.05) -> Resolver:
    """Adapter for spatialAmigo (``spatialamigo.Resolver(Scene(...)).resolve(text).best``).

    Raises :class:`ResolutionError` when nothing matches or the best candidate wins by less
    than ``min_margin``.
    """
    import spatialamigo as sa

    def resolve(scene: dict, instruction: str) -> str:
        try:
            res = sa.Resolver(sa.Scene(scene)).resolve(instruction)
        except sa.SpatialAmigoError as err:
            raise ResolutionError(str(err)) from err
        if res.best is None:
            raise ResolutionError(f"spatialAmigo found no object for {instruction!r}")
        if res.margin < min_margin:
            raise ResolutionError(f"{instruction!r} is ambiguous: {res.explanation}")
        return res.best

    return resolve


def default_resolver() -> Resolver:
    """spatialAmigo when it is installed, the built-in resolver otherwise."""
    try:
        return spatialamigo_resolver()
    except ImportError:
        return builtin_resolver


def plan_from_instruction(
    scene: SceneLike,
    instruction: str,
    *,
    robot: Robot,
    start: Any,
    resolver: Optional[Resolver] = None,
    **pick_kwargs: Any,
) -> InstructionResult:
    """Resolves the object an instruction refers to and plans picking it with :func:`plan_pick`.

    ``pick_kwargs`` (place, clearance, grasp_depth, seed) are passed on to :func:`plan_pick`.
    """
    data = load_scene(scene)
    object_id = (resolver or default_resolver())(data, instruction)
    return InstructionResult(object_id, plan_pick(robot, data, object_id, start, **pick_kwargs))
