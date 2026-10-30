"""The finite-element check against a case with a known answer: a cantilevered plate."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("pytetwild")
pytest.importorskip("skfem")
pytest.importorskip("pypardiso")

from cad.fea import Model, Patch  # noqa: E402
from cad.lib import box  # noqa: E402


def test_cantilever_bending_stress():
    # 100 x 20 x 6 mm plate, held at x = 0, 100 N down at the free end: beam theory gives
    # sigma = M c / I = F (L - x) (t / 2) / (w t^3 / 12) at the surface, 41.7 MPa at x = 50
    m = Model(box((100, 20, 6), (50, 0, 0)), "al6061", size=2.0)
    held = [Patch("face", (0, 0, 0), (1, 0, 0), 0, 20)]
    tip = [Patch("face", (100, 0, 0), (1, 0, 0), 0, 20)]
    vm, near = m.solve(held, tip, (0, 0, -100.0), (0, 0, 0), (100, 0, 0))
    c = m.centroids()
    mid = (np.abs(c[:, 0] - 50) < 3) & ~near
    assert vm[mid].max() == pytest.approx(41.7, rel=0.12)


def test_thin_strip_torsion():
    # 150 x 56 x 6 mm strip twisted by 127 N·m: tau = T / (alpha b t^2), alpha 0.31 for b/t 9.3,
    # 203 MPa; von Mises sqrt(3) tau, 352 MPa (the hip roll bracket's front plate, decisions
    # 2026-10-30)
    m = Model(box((150, 56, 6), (75, 0, 0)), "al6061", size=3.0)
    held = [Patch("face", (0, 0, 0), (1, 0, 0), 0, 40)]
    end = [Patch("face", (150, 0, 0), (1, 0, 0), 0, 40)]
    vm, _ = m.solve(held, end, (0, 0, 0), (127000.0, 0, 0), (150, 0, 0))
    mid = np.abs(m.centroids()[:, 0] - 75) < 10
    assert vm[mid].max() == pytest.approx(352, rel=0.12)
