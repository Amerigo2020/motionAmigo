import json
import sys
import types
from pathlib import Path

import pytest

import motionamigo as ma

SCENE = Path(__file__).resolve().parents[3] / "examples" / "scenes" / "tabletop.json"
DATA = json.loads(SCENE.read_text(encoding="utf-8"))


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("the mug left of the laptop", "mug_2"),
        ("pick up the mug to the right of the laptop", "mug_1"),
        ("the mug next to the bowl", "mug_1"),
        ("the mug behind the laptop", "mug_2"),
        ("the mug in front of the laptop", "mug_1"),
        ("grab the bowl", "bowl_1"),
        ("pick mug_2", "mug_2"),
    ],
)
def test_builtin_resolver(text, expected):
    assert ma.builtin_resolver(DATA, text) == expected


@pytest.mark.parametrize("text", ["the mug", "the spoon", "the bowl left of the laptop"])
def test_builtin_resolver_errors(text):
    with pytest.raises(ma.ResolutionError):
        ma.builtin_resolver(DATA, text)


def test_builtin_resolver_without_viewpoint_looks_along_x():
    data = {k: v for k, v in DATA.items() if k != "viewpoint"}
    assert ma.builtin_resolver(data, "the mug left of the laptop") == "mug_2"


def test_plan_from_instruction_with_custom_resolver():
    calls = []

    def resolver(scene, instruction):
        calls.append(instruction)
        return "mug_1"

    out = ma.plan_from_instruction(SCENE, "whatever", robot=ma.Robot.panda(), start=ma.PANDA_READY, resolver=resolver)
    assert calls == ["whatever"]
    assert out.object_id == "mug_1"
    assert len(out.pick.approach) > 0


def test_plan_from_instruction_builtin():
    out = ma.plan_from_instruction(
        SCENE,
        "the mug right of the laptop",
        robot=ma.Robot.panda(),
        start=ma.PANDA_READY,
        resolver=ma.builtin_resolver,
    )
    assert out.object_id == "mug_1"


def _stub(best, margin=1.0):
    sa = types.ModuleType("spatialamigo")

    class SpatialAmigoError(ValueError):
        pass

    class Resolution:
        def __init__(self):
            self.best, self.margin, self.explanation = best, margin, "stub"

    class Resolver:
        def __init__(self, scene):
            assert scene.data["version"] == "0.1"

        def resolve(self, query):
            if query == "bad":
                raise SpatialAmigoError("cannot parse")
            return Resolution()

    sa.SpatialAmigoError = SpatialAmigoError
    sa.Scene = lambda data: types.SimpleNamespace(data=data)
    sa.Resolver = Resolver
    return sa


def test_spatialamigo_adapter(monkeypatch):
    monkeypatch.setitem(sys.modules, "spatialamigo", _stub("mug_2"))
    resolver = ma.default_resolver()
    assert resolver is not ma.builtin_resolver
    assert resolver(DATA, "the mug left of the laptop") == "mug_2"
    with pytest.raises(ma.ResolutionError):
        resolver(DATA, "bad")


@pytest.mark.parametrize(("best", "margin"), [(None, 1.0), ("mug_1", 0.01)])
def test_spatialamigo_adapter_rejects(monkeypatch, best, margin):
    monkeypatch.setitem(sys.modules, "spatialamigo", _stub(best, margin))
    with pytest.raises(ma.ResolutionError):
        ma.spatialamigo_resolver()(DATA, "the mug")


def test_default_resolver_falls_back(monkeypatch):
    monkeypatch.setitem(sys.modules, "spatialamigo", None)
    assert ma.default_resolver() is ma.builtin_resolver
