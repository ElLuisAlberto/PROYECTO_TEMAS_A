"""Check that the reconstructed lab matches the simulated world's geometry."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

try:
    from teleop_cooperativo.desktop_qt import (
        FINGER_ORIGINS,
        LAB_FLOOR_TOP,
        LAB_LEG_CENTERS,
        LAB_LEG_SIZE,
        LAB_TABLE_CENTER,
        LAB_TABLE_SIZE,
        MESH_DIR,
        _load_mesh,
    )
except ImportError as error:
    raise unittest.SkipTest(f"Qt/VTK dependencies not installed: {error}")


ROOT = Path(__file__).resolve().parents[2]


def vector(element):
    return tuple(float(value) for value in element.text.split())


class ReconstructedLabTest(unittest.TestCase):
    def test_table_and_floor_match_gazebo_collision_geometry(self):
        world = ET.parse(ROOT / "ros2_ws/src/course_teleop_simulation/worlds/jaco2_lab.sdf")
        ground = world.find(".//model[@name='ground_plane']/pose")
        table = world.find(".//model[@name='work_table']/link")
        self.assertEqual(vector(ground)[2], LAB_FLOOR_TOP)
        top = table.find("collision[@name='top_collision']")
        self.assertEqual(vector(top.find("pose"))[:3], LAB_TABLE_CENTER)
        self.assertEqual(vector(top.find("geometry/box/size")), LAB_TABLE_SIZE)
        legs = [item for item in table.findall("collision") if item.attrib["name"].startswith("leg_")]
        self.assertEqual({vector(leg.find("pose"))[:3] for leg in legs}, set(LAB_LEG_CENTERS))
        for leg in legs:
            self.assertEqual(vector(leg.find("geometry/box/size")), LAB_LEG_SIZE)

    def test_fingers_use_original_fixed_zero_pose_from_xacro(self):
        document = ET.parse(ROOT / "ros2_ws/src/course_teleop_description/urdf/kinova_finger_set.xacro")
        namespace = "{http://www.ros.org/wiki/xacro}"
        fingers = document.findall(f".//{namespace}macro[@name='kinova_3fingers']/{namespace}kinova_finger")
        actual = tuple((tuple(map(float, finger.attrib["finger_origin_xyz"].split())),
                        tuple(map(float, finger.attrib["finger_origin_rpy"].split())))
                       for finger in fingers)
        self.assertEqual(actual, FINGER_ORIGINS)

    def test_original_dae_material_colors_survive_import(self):
        for name, expected in (("base", 0.1788232), ("ring_big", 0.627451),
                               ("finger_proximal", 0.5960783)):
            parts = _load_mesh(MESH_DIR / f"{name}.dae")
            self.assertTrue(parts)
            for mesh, color, _ in parts:
                self.assertGreater(mesh.GetNumberOfPolys(), 0)
                self.assertAlmostEqual(color[0], expected, places=6)


if __name__ == "__main__":
    unittest.main()
