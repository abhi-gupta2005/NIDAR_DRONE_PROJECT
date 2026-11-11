"""
Static geometry checks of the NIDAR Gazebo models (2026-11-02c).

The first rig had a visual block in the LiDAR scan plane; Gazebo's gpu_lidar
saw it as an obstacle 0.16 m straight ahead -- exactly where the camera target
is, so V18 would have fused the person with a 0.16 m range. These tests keep
every primitive visual out of the scan plane and the x500 meshes below it.
Mesh heights: x500 frame (NXP-HGD-CF.dae) tops out 0.051 m above base_link
(measured 2026-11-02c); props ~0.044-0.06 m.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

MODELS = Path(__file__).resolve().parents[1] / "gazebo" / "models"
X500_MESH_TOP_ABOVE_ORIGIN = 0.03        # frame mesh max z 0.0264 in its own frame (+ visual offset in pose)


def pose(el):
    txt = el.findtext("pose")
    return [float(v) for v in txt.split()] if txt else [0.0] * 6


def half_height(geom):
    box = geom.find("box")
    if box is not None:
        return float(box.findtext("size").split()[2]) / 2
    cyl = geom.find("cylinder")
    if cyl is not None:
        return float(cyl.findtext("length")) / 2
    sph = geom.find("sphere")
    if sph is not None:
        return float(sph.findtext("radius"))
    return None                                            # mesh


def visuals_relative_to_lidar(model_dir):
    root = ET.parse(MODELS / model_dir / "model.sdf").getroot().find("model")
    links = {l.get("name"): l for l in root.findall("link")}
    lidar_link = next(l for l in links.values() if l.find("sensor[@type='gpu_lidar']") is not None)
    sensor = lidar_link.find("sensor[@type='gpu_lidar']")
    plane_z = pose(lidar_link)[2] + pose(sensor)[2]
    out = []
    for name, link in links.items():
        lz = pose(link)[2]
        for v in link.findall("visual"):
            out.append((name, v.get("name"), lz + pose(v)[2] - plane_z, v.find("geometry")))
    return plane_z, out


@pytest.mark.parametrize("model", ["nidar_rig", "x500_nidar_airmouse"])
def test_no_primitive_visual_in_the_lidar_plane(model):
    plane_z, vis = visuals_relative_to_lidar(model)
    for link, name, dz, geom in vis:
        h = half_height(geom)
        if h is None:
            continue
        assert abs(dz) >= h + 0.003, f"{model}/{name}: spans {dz - h:+.3f}..{dz + h:+.3f} m around the scan plane"


def test_rig_x500_meshes_stay_below_the_plane():
    plane_z, vis = visuals_relative_to_lidar("nidar_rig")
    meshes = [(n, dz) for _, n, dz, g in vis if g.find("mesh") is not None]
    assert len(meshes) == 13, "frame + 4 motor bases + 4 props + 4 bells"
    for name, dz in meshes:
        assert dz + X500_MESH_TOP_ABOVE_ORIGIN <= -0.03, f"{name} top at {dz + X500_MESH_TOP_ABOVE_ORIGIN:+.3f} m"


def test_rig_and_drone_sensor_geometry_match():
    def sensors(model):
        root = ET.parse(MODELS / model / "model.sdf").getroot().find("model")
        res = {}
        for link in root.findall("link"):
            lp = pose(link)
            for s in link.findall("sensor"):
                sp = pose(s)
                res[s.get("type")] = (lp[0] + sp[0], lp[1] + sp[1], lp[2] + sp[2], s)
        return res
    rig, drone = sensors("nidar_rig"), sensors("x500_nidar_airmouse")
    for kind in ("gpu_lidar", "camera"):
        assert rig[kind][3].findtext("topic") == drone[kind][3].findtext("topic")
    # camera relative to LiDAR identical on both
    for i in range(3):
        assert rig["camera"][i] - rig["gpu_lidar"][i] == pytest.approx(drone["camera"][i] - drone["gpu_lidar"][i])
    hfov = float(drone["camera"][3].find("camera").findtext("horizontal_fov"))
    assert math.degrees(hfov) == pytest.approx(70.0, abs=0.01)
