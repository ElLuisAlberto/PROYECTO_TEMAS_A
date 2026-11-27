"""Qt base-station viewer: Jaco2 link meshes driven only by gateway telemetry.

This reconstructs arm posture, not a camera image of the Gazebo world.
"""
from __future__ import annotations

import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import collada
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

from . import jaco2_kinematics as kin
from .cad_camera import CadCameraStyle, CadVTKWidget
from .robot_frames import FINGER_ORIGINS, link_transforms, _origin_transform, finger_link_transform
from .virtual_task_spec import OBJECTS, INITIAL, GOALS, GOAL_HEIGHTS, _parts

MESH_DIR = (Path(__file__).resolve().parents[2] / "ros2_ws/src/course_teleop_description"
            / "meshes/kinova_j2n6s300")
LAB_WORLD = Path(__file__).resolve().parents[2] / "ros2_ws/src/thesis_simulation/worlds/jaco2_workstation.sdf"
WINDOW_PHOTO = Path(__file__).resolve().parents[2] / "assets/ventana_pucp.jpeg"
MESHES = ("base", "shoulder", "arm", "forearm", "wrist", "wrist", "hand_3finger")
STALE_SEC = 1.0
LAB_TABLE_CENTER = (0.30, 0.0, -0.03)
LAB_TABLE_SIZE = (1.20, 0.90, 0.06)
LAB_LEG_CENTERS = tuple((x, y, -0.39) for x in (-0.22, 0.82) for y in (-0.38, 0.38))
LAB_LEG_SIZE = (0.07, 0.07, 0.66)
LAB_FLOOR_TOP = -0.72

# These fixed finger poses reproduce the zero-angle joints from the Kinova
# Xacro. Finger motion is intentionally not inferred from six arm joints.
def _load_mesh(path):
    """Load each DAE primitive separately, preserving its authored material."""
    document = collada.Collada(str(path))
    parts = []
    for geometry in document.scene.objects("geometry"):
        for primitive in geometry.primitives():
            if not hasattr(primitive, "vertex_index") or primitive.vertex_index.shape[1] != 3:
                raise ValueError(f"La malla no contiene triángulos: {path.name}")
            vertices = np.ascontiguousarray(primitive.vertex, dtype=np.float64)
            indices = np.asarray(primitive.vertex_index, dtype=np.int64)
            cells = np.empty((len(indices), 4), dtype=np.int64)
            cells[:, 0] = 3
            cells[:, 1:] = indices
            points = vtk.vtkPoints()
            points.SetData(numpy_to_vtk(vertices, deep=True))
            polygons = vtk.vtkCellArray()
            polygons.SetCells(len(indices), numpy_to_vtkIdTypeArray(cells.ravel(), deep=True))
            data = vtk.vtkPolyData()
            data.SetPoints(points)
            data.SetPolys(polygons)
            normals = vtk.vtkPolyDataNormals()
            normals.SetInputData(data)
            normals.SplittingOff()
            normals.ConsistencyOn()
            normals.Update()
            material = getattr(primitive, "material", None)
            effect = getattr(material, "effect", None)
            diffuse = getattr(effect, "diffuse", None)
            specular = getattr(effect, "specular", None)
            # The project DAE files have RGBA values. Avoid mistaking a
            # texture-map object for a color if other meshes are used later.
            color = tuple(diffuse[:3]) if isinstance(diffuse, tuple) and len(diffuse) >= 3 else (0.55, 0.55, 0.55)
            shine = tuple(specular[:3]) if isinstance(specular, tuple) and len(specular) >= 3 else (0.3, 0.3, 0.3)
            parts.append((normals.GetOutput(), color, shine))
    if not parts:
        raise ValueError(f"Malla vacía: {path.name}")
    return parts


class RobotView(QtWidgets.QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("viewer")
        layout = QtWidgets.QVBoxLayout(self)
        heading = QtWidgets.QHBoxLayout()
        self.label = QtWidgets.QLabel("JACO2 · VISTA RECONSTRUIDA")
        self.label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        heading.addWidget(self.label, 1)
        reset = QtWidgets.QPushButton("Restablecer vista")
        reset.setToolTip("Restablece la cámara del laboratorio")
        reset.clicked.connect(self.reset_view)
        heading.addWidget(reset)
        layout.addLayout(heading)
        self.widget = CadVTKWidget(self)
        navigation_help = (
            "Botón izquierdo: girar alrededor del punto seleccionado.\n"
            "Botón central: desplazar. Mayús + botón central: girar.\n"
            "Rueda: acercar o alejar hacia el cursor.\n"
            "Restablecer vista recupera la cámara inicial."
        )
        self.widget.setToolTip(navigation_help)
        self.label.setToolTip(navigation_help)
        layout.addWidget(self.widget, 1)
        self.status = QtWidgets.QLabel("Esperando telemetría del brazo…")
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(self.status)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.92, 0.93, 0.97)
        self.renderer.SetBackground2(0.98, 0.98, 1.0)
        self.renderer.GradientBackgroundOn()
        self.widget.GetRenderWindow().AddRenderer(self.renderer)
        self.camera_style = CadCameraStyle(self.renderer, self.widget.devicePixelRatioF)
        self.widget.SetInteractorStyle(self.camera_style)
        self.renderer.SetAmbient(0.48, 0.48, 0.52)
        light = vtk.vtkLight()
        light.SetLightTypeToSceneLight()
        light.SetPosition(-2, -3, 5)
        light.SetFocalPoint(0.2, 0, 0.4)
        light.SetColor(1.0, 1.0, 1.0)
        light.SetIntensity(0.95)
        self.renderer.AddLight(light)
        fill = vtk.vtkLight()
        fill.SetLightTypeToSceneLight()
        fill.SetPosition(2, 2, 3)
        fill.SetFocalPoint(0, 0, 0.5)
        fill.SetColor(0.90, 0.93, 1.0)
        fill.SetIntensity(0.5)
        self.renderer.AddLight(fill)

        # Keep the work surface and robot origin aligned with the lab model.
        self._wood_texture = self._make_wood_texture()
        self._box(LAB_TABLE_CENTER, LAB_TABLE_SIZE, (1, 1, 1), texture=self._wood_texture)
        for center in LAB_LEG_CENTERS:
            self._box(center, LAB_LEG_SIZE, (0.25, 0.20, 0.17))
        self._box((0.1, 0.0, LAB_FLOOR_TOP - 0.01), (5.0, 5.0, 0.02), (0.72, 0.70, 0.67))
        self._add_tile_floor()
        self._add_lab_decor()
        self._add_lab_furniture()

        self.actors = []
        self._robot_parts = []
        cache = {}

        def add_mesh(index, name, offset=None, finger_index=None, distal=False):
            if name not in cache:
                cache[name] = _load_mesh(MESH_DIR / f"{name}.dae")
            for data, color, specular in cache[name]:
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(data)
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                actor.GetProperty().SetColor(*color)
                actor.GetProperty().SetSpecularColor(*specular)
                actor.GetProperty().SetAmbient(0.32)
                actor.GetProperty().SetDiffuse(0.75)
                actor.GetProperty().SetSpecular(0.35)
                actor.GetProperty().SetSpecularPower(45)
                actor.SetVisibility(False)
                self.renderer.AddActor(actor)
                self.actors.append(actor)
                self._robot_parts.append((actor, index, np.eye(4) if offset is None else offset, finger_index, distal))

        for index, name in enumerate(MESHES):
            add_mesh(index, name)
        for index in (1, 2, 3):
            add_mesh(index, "ring_big")
        for index in (4, 5, 6):
            add_mesh(index, "ring_small")
        for finger_index, (translation, rpy) in enumerate(FINGER_ORIGINS):
            finger = _origin_transform(translation, rpy)
            add_mesh(6, "finger_proximal", finger, finger_index)
            add_mesh(6, "finger_distal", finger, finger_index, distal=True)

        self._init_virtual_task()
        self.reset_view(render=False)
        self.widget.Initialize()

    def _init_virtual_task(self):
        self._task_parts = []
        self._task_goals = []
        for index, (spec, goal, height) in enumerate(zip(OBJECTS, GOALS, GOAL_HEIGHTS)):
            if height > 0:
                pedestal = self._box((goal[0], goal[1], height / 2), (.094, .094, height), (.72, .75, .79))
                pedestal.SetVisibility(False); pedestal.PickableOff(); self._task_goals.append(pedestal)
            disk = vtk.vtkRegularPolygonSource()
            disk.SetNumberOfSides(48); disk.SetRadius(.045); disk.SetCenter(*goal); disk.SetNormal(0, 0, 1)
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(disk.GetOutputPort())
            target = vtk.vtkActor(); target.SetMapper(mapper); target.GetProperty().SetColor(*spec['color'])
            target.GetProperty().SetOpacity(.48); target.SetVisibility(False); target.PickableOff()
            self.renderer.AddActor(target); self._task_goals.append(target)
            for part_index, part in enumerate(_parts(spec)):
                kind = part['shape']
                if kind == 'box':
                    source = vtk.vtkCubeSource()
                    source.SetXLength(2*part['half'][0]); source.SetYLength(2*part['half'][1]); source.SetZLength(2*part['half'][2])
                elif kind == 'sphere':
                    source = vtk.vtkSphereSource(); source.SetRadius(part['radius']); source.SetThetaResolution(20); source.SetPhiResolution(16)
                elif kind == 'capsule':
                    source = vtk.vtkCapsuleSource(); source.SetRadius(part['radius']); source.SetCylinderLength(part['height']); source.SetThetaResolution(20); source.SetPhiResolution(16)
                else:
                    source = vtk.vtkCylinderSource(); source.SetRadius(part['radius']); source.SetHeight(part['height']); source.SetResolution(24)
                mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(source.GetOutputPort())
                actor = vtk.vtkActor(); actor.SetMapper(mapper)
                color = tuple(min(1.0, c + .13) for c in spec['color']) if part_index else spec['color']
                actor.GetProperty().SetColor(*color); actor.GetProperty().SetAmbient(.35); actor.GetProperty().SetDiffuse(.65)
                actor.SetVisibility(False); actor.PickableOff(); self.renderer.AddActor(actor)
                local = np.eye(4); local[:3, 3] = part['center']
                if kind in ('cylinder', 'capsule'):
                    local[:3, :3] = np.array([[1,0,0], [0,0,-1], [0,1,0]])
                self._task_parts.append((actor, index, local))
            for eye_x in (-.006, .006):
                eye = vtk.vtkSphereSource(); eye.SetRadius(.0023); eye.SetThetaResolution(12); eye.SetPhiResolution(10)
                mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(eye.GetOutputPort())
                actor = vtk.vtkActor(); actor.SetMapper(mapper); actor.GetProperty().SetColor(.08, .09, .13)
                actor.SetVisibility(False); actor.PickableOff(); self.renderer.AddActor(actor)
                local = np.eye(4); local[:3, 3] = (eye_x, -.0165, .035)
                self._task_parts.append((actor, index, local))

    @staticmethod
    def _task_pose(position, quaternion):
        x, y, z, w = quaternion
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        transform = np.eye(4)
        transform[:3, :3] = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        transform[:3, 3] = position
        return transform

    def update_task(self, task):
        if task is None:
            for actor, _, _ in self._task_parts: actor.SetVisibility(False)
            for actor in self._task_goals: actor.SetVisibility(False)
            return
        poses = {item['id']: self._task_pose(item['position'], item['quaternion_xyzw']) for item in task['objects']}
        for actor, index, local in self._task_parts:
            transform = poses[index] @ local
            matrix = vtk.vtkMatrix4x4()
            for row in range(4):
                for column in range(4): matrix.SetElement(row, column, float(transform[row, column]))
            actor.SetUserMatrix(matrix); actor.SetVisibility(True)
        for actor in self._task_goals: actor.SetVisibility(True)

    @staticmethod
    def _make_wood_texture():
        """Create a subtle, repeatable wood grain for the existing furniture."""
        rng = np.random.default_rng(38)
        y, x = np.mgrid[0:256, 0:256]
        grain = np.sin(y * 0.22 + 2.0 * np.sin(x * 0.018 + y * 0.014))
        grain += 0.35 * np.sin(y * 0.71 + np.sin(x * 0.025))
        grain += rng.normal(0, 0.18, (256, 256))
        base = np.array((153, 105, 68), dtype=float)
        rgb = np.clip(base + grain[..., None] * np.array((5, 4, 3)), 0, 255).astype(np.uint8)
        data = vtk.vtkImageData()
        data.SetDimensions(256, 256, 1)
        data.GetPointData().SetScalars(numpy_to_vtk(rgb.reshape(-1, 3), deep=True, array_type=vtk.VTK_UNSIGNED_CHAR))
        texture = vtk.vtkTexture()
        texture.SetInputData(data)
        texture.InterpolateOn()
        texture.RepeatOn()
        return texture

    def _box(self, center, dimensions, color, *, texture=None):
        cube = vtk.vtkCubeSource()
        cube.SetCenter(*center)
        cube.SetXLength(dimensions[0]); cube.SetYLength(dimensions[1]); cube.SetZLength(dimensions[2])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetAmbient(0.30)
        actor.GetProperty().SetDiffuse(0.70)
        actor.GetProperty().SetSpecular(0.12)
        actor.GetProperty().SetSpecularPower(24)
        if texture is not None:
            actor.SetTexture(texture)
        self.renderer.AddActor(actor)
        return actor

    def _add_tile_floor(self):
        # Fine recessed grout lines and alternating off-white glazed tiles.
        for i in range(12):
            for j in range(12):
                x, y = -2.2 + i * 0.42, -2.2 + j * 0.42
                shade = 0.88 if (i + j) % 2 else 0.91
                self._box((x, y, LAB_FLOOR_TOP + 0.001), (0.414, 0.414, 0.002),
                          (shade, shade * 0.986, shade * 0.96))

    def _add_lab_furniture(self):
        # A wooden work desk with drawers, metal handles and a desktop monitor.
        wood = self._wood_texture
        self._box((-1.35, 1.25, -0.02), (0.95, 0.58, 0.05), (1, 1, 1), texture=wood)
        self._box((-1.35, 1.25, -0.10), (0.88, 0.52, 0.11), (0.43, 0.29, 0.19))
        for x in (-1.76, -0.94):
            for y in (1.02, 1.48):
                self._box((x, y, -0.41), (0.045, 0.045, 0.62), (0.25, 0.23, 0.21))
        self._box((-1.65, 1.19, -0.34), (0.27, 0.45, 0.48), (0.49, 0.34, 0.23))
        for z in (-0.19, -0.33, -0.47):
            self._box((-1.65, 0.956, z), (0.25, 0.012, 0.125), (0.56, 0.40, 0.27))
            self._box((-1.65, 0.945, z), (0.09, 0.013, 0.008), (0.65, 0.66, 0.64))
        self._box((-1.32, 1.38, 0.015), (0.28, 0.18, 0.018), (0.10, 0.11, 0.12))
        self._box((-1.32, 1.42, 0.105), (0.035, 0.035, 0.18), (0.13, 0.14, 0.15))
        self._box((-1.32, 1.42, 0.29), (0.42, 0.04, 0.25), (0.10, 0.11, 0.13))
        self._box((-1.32, 1.397, 0.29), (0.38, 0.006, 0.21), (0.14, 0.19, 0.21))
        self._box((-1.32, 1.05, 0.015), (0.36, 0.13, 0.014), (0.14, 0.14, 0.14))
        # Baseboards and two recognizable socket plates replace the
        # ambiguous horizontal block on the wall.
        self._box((0, 2.14, -0.67), (5.3, 0.025, 0.09), (0.60, 0.58, 0.54))
        self._box((-2.65, 0, -0.67), (0.025, 4.25, 0.09), (0.60, 0.58, 0.54))
        for x in (0.55, 1.25):
            self._box((x, 2.112, -0.43), (0.11, 0.015, 0.075), (0.90, 0.89, 0.86))
            for dx in (-0.022, 0.022):
                self._box((x + dx, 2.102, -0.43), (0.006, 0.003, 0.025), (0.25, 0.25, 0.24))
        # A compact task chair suggests the purpose of the workstation.
        self._box((-1.42, 0.60, -0.33), (0.38, 0.36, 0.07), (0.22, 0.25, 0.27))
        self._box((-1.42, 0.44, -0.14), (0.38, 0.065, 0.35), (0.22, 0.25, 0.27))
        self._box((-1.42, 0.60, -0.53), (0.04, 0.04, 0.34), (0.34, 0.35, 0.36))
        self._box((-1.42, 0.60, -0.69), (0.48, 0.05, 0.025), (0.25, 0.26, 0.27))
        self._box((-1.42, 0.60, -0.69), (0.05, 0.43, 0.025), (0.25, 0.26, 0.27))

    def _add_lab_decor(self):
        """Reconstruct the existing SDF classroom as a visual backdrop."""
        world = ET.parse(LAB_WORLD)
        visuals = world.findall(".//model[@name='academic_lab_decor']/link/visual")
        for visual in visuals:
            # The camera looks into the room from this side; omit the near
            # wall so it cannot obscure the arm and table.
            name = visual.get("name", "")
            if (name == "right_wall" or name == "window_glass"
                    or name.startswith("window_landscape_")
                    or name.startswith("window_tree_")
                    or name.startswith("whiteboard_")
                    or name == "back_wall_lab_band"
                    or name.startswith("lab_bench_")
                    or name.startswith("lab_cabinet_")):
                continue
            pose = [float(v) for v in (visual.findtext("pose") or "0 0 0 0 0 0").split()]
            box = visual.find("geometry/box/size")
            sphere = visual.find("geometry/sphere/radius")
            cylinder = visual.find("geometry/cylinder")
            if box is not None:
                shape = vtk.vtkCubeSource()
                x, y, z = (float(v) for v in box.text.split())
                shape.SetXLength(x); shape.SetYLength(y); shape.SetZLength(z)
            elif sphere is not None:
                shape = vtk.vtkSphereSource()
                shape.SetRadius(float(sphere.text))
                shape.SetThetaResolution(20); shape.SetPhiResolution(16)
            elif cylinder is not None:
                shape = vtk.vtkCylinderSource()
                shape.SetRadius(float(cylinder.findtext("radius")))
                shape.SetHeight(float(cylinder.findtext("length")))
                shape.SetResolution(16)
                # VTK cylinders point along Y, while SDF cylinders use Z.
                pose[3] += math.pi / 2
            else:
                continue
            color = visual.findtext("material/diffuse") or "0.7 0.7 0.7 1"
            rgb = [float(v) for v in color.split()[:3]]
            if name.endswith("wall"):
                rgb = [0.76, 0.76, 0.74]
            elif name.startswith("window_frame_") or name == "window_mullion":
                rgb = [0.29, 0.13, 0.07]
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(shape.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*rgb)
            actor.GetProperty().SetAmbient(0.42)
            # Compress the decorative 10 m room around the original table so
            # the classroom and robot remain visible in the default view.
            transform = vtk.vtkTransform()
            transform.Translate(0, 0, LAB_FLOOR_TOP)
            transform.Scale(0.55, 0.45, 0.8)
            transform.Translate(*pose[:3])
            transform.RotateZ(math.degrees(pose[5]))
            transform.RotateY(math.degrees(pose[4]))
            transform.RotateX(math.degrees(pose[3]))
            actor.SetUserTransform(transform)
            actor.PickableOff()
            self.renderer.AddActor(actor)

        # The project's original PUCP photo is mapped to the window opening.
        reader = vtk.vtkJPEGReader()
        reader.SetFileName(str(WINDOW_PHOTO))
        texture = vtk.vtkTexture()
        texture.SetInputConnection(reader.GetOutputPort())
        texture.InterpolateOn()
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-3.61, 4.785, 1.40)
        plane.SetPoint1(-1.79, 4.785, 1.40)
        plane.SetPoint2(-3.61, 4.785, 2.50)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(plane.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetTexture(texture)
        actor.GetProperty().SetAmbient(1.0)
        actor.GetProperty().SetDiffuse(0.0)
        transform = vtk.vtkTransform()
        transform.Translate(0, 0, LAB_FLOOR_TOP)
        transform.Scale(0.55, 0.45, 0.8)
        actor.SetUserTransform(transform)
        actor.PickableOff()
        self.renderer.AddActor(actor)

    def reset_view(self, _checked=False, *, render=True):
        self.camera_style.cancel_drag()
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(1.8, -2.8, 1.35)
        camera.SetFocalPoint(-0.15, 0.3, 0.60)
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(1.4)
        self.renderer.ResetCameraClippingRange()
        if render:
            self.widget.GetRenderWindow().Render()

    def update_joints(self, joints, fingers=None, finger_source=None):
        if joints is None:
            for actor in self.actors:
                actor.SetVisibility(False)
            self.status.setText("SIN TELEMETRÍA · Vista no actualizada")
        else:
            try:
                transforms = link_transforms(joints)
            except (TypeError, ValueError):
                self.update_joints(None)
                return
            for actor, index, offset, finger_index, distal in self._robot_parts:
                transform = transforms[index] @ offset
                if finger_index is not None:
                    angle = 0.0 if fingers is None else fingers[finger_index]
                    transform = finger_link_transform(transforms[index], offset, angle, distal)
                matrix = vtk.vtkMatrix4x4()
                for row in range(4):
                    for column in range(4):
                        matrix.SetElement(row, column, float(transform[row, column]))
                actor.SetUserMatrix(matrix)
                actor.SetVisibility(True)
            finger_status = "SIMULADOS" if fingers is not None and finger_source == "simulated" else "físicos" if fingers is not None and finger_source == "physical" else "sin telemetría verificada"
            self.status.setText("EN VIVO · Vista reconstruida · Dedos " + finger_status)
        self.renderer.ResetCameraClippingRange()
        self.widget.GetRenderWindow().Render()


class RecoveryDialog(QtWidgets.QDialog):
    """Local hold-to-move controls; loss of focus always releases the jog."""
    def __init__(self, parent, control):
        super().__init__(parent)
        self.control = control
        self.held = None
        self.setWindowTitle("JACO2 · Recuperación por articulaciones")
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)
        self.setMinimumWidth(390)
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("RECUPERACIÓN POR ARTICULACIONES")
        title.setObjectName("eyebrow")
        layout.addWidget(title)
        help_label = QtWidgets.QLabel("Mantén pulsado − o + para mover un joint. Suelta para detener.")
        help_label.setWordWrap(True)
        layout.addWidget(help_label)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Velocidad angular"))
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0.1, 5.0)
        self.speed.setSingleStep(0.5)
        self.speed.setValue(2.0)
        self.speed.setSuffix(" °/s")
        row.addWidget(self.speed)
        layout.addLayout(row)
        grid = QtWidgets.QGridLayout()
        self.positions = []
        self.buttons = []
        for i in range(6):
            grid.addWidget(QtWidgets.QLabel(f"Joint {i + 1}"), i, 0)
            position = QtWidgets.QLabel("—")
            self.positions.append(position)
            grid.addWidget(position, i, 1)
            for column, sign, text in ((2, -1, "−"), (3, 1, "+")):
                button = QtWidgets.QPushButton(text)
                button.setMinimumHeight(36)
                button.installEventFilter(self)
                button.pressed.connect(lambda i=i, sign=sign: self.press(i, sign))
                button.released.connect(self.stop)
                grid.addWidget(button, i, column)
                self.buttons.append(button)
        layout.addLayout(grid)
        self.status = QtWidgets.QLabel("Esperando modo de recuperación…")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        close = QtWidgets.QPushButton("Cerrar")
        close.clicked.connect(self.close)
        layout.addWidget(close)

    def press(self, index, sign):
        if self.isActiveWindow() and self.buttons[index * 2 + (sign > 0)].isEnabled():
            self.held = (index, sign)

    def stop(self):
        if self.held is not None:
            self.held = None
            self.control.joint_velocity([0.0] * 6)

    def tick(self, allowed, joints):
        allowed = allowed and self.isVisible() and self.isActiveWindow()
        if not allowed:
            self.stop()
        for button in self.buttons:
            button.setEnabled(allowed)
        for i, label in enumerate(self.positions):
            label.setText(f"{math.degrees(joints[i]):.1f}°" if joints is not None else "—")
        self.status.setText("Modo activo · web pausada" if allowed else "Movimiento bloqueado · espera estado operativo")
        if allowed and self.held is not None:
            values = [0.0] * 6
            index, sign = self.held
            values[index] = sign * math.radians(self.speed.value())
            self.control.joint_velocity(values)

    def eventFilter(self, watched, event):
        if watched in self.buttons and event.type() in (QtCore.QEvent.FocusOut, QtCore.QEvent.UngrabMouse, QtCore.QEvent.Hide):
            self.stop()
        return super().eventFilter(watched, event)

    def event(self, event):
        if event.type() in (QtCore.QEvent.WindowDeactivate, QtCore.QEvent.Hide, QtCore.QEvent.Close):
            self.stop()
        return super().event(event)


class QtDesktopConsole:
    """Fullscreen observer UI with an optional local direct-output switch."""

    def __init__(self, state, observer, direct_bridge=False, physical=False):
        self.state, self.observer = state, observer
        self.physical = physical
        self.recovery_dialog = None
        self.shutdown_locked = False
        self.shutdown_started = 0.0
        self.output_control = None
        if direct_bridge:
            from .direct_output_control import DirectOutputControl
            self.output_control = DirectOutputControl()
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("JACO2 · Estación base")
        self.window.setMinimumSize(1, 1)
        self.window.setStyleSheet("""
            QMainWindow, QWidget#root { background: #eef1f6; }
            QWidget { color: #272641; font-family: "Inter", "Segoe UI", "DejaVu Sans", sans-serif; font-size: 13px; }
            QLabel { background: transparent; border: none; }
            QFrame#viewer, QFrame#card {
                background: #ffffff; border: 1px solid #dfe4ed; border-radius: 12px;
            }
            QLabel#brand { font-size: 22px; font-weight: 750; color: #3935a5; }
            QLabel#eyebrow { font-size: 11px; font-weight: 700; letter-spacing: 1px; color: #66718a; }
            QLabel#section { font-size: 15px; font-weight: 700; color: #272641; }
            QLabel#caption { font-size: 12px; color: #59657a; }
            QLabel#value { font-family: "JetBrains Mono", "Consolas", monospace; font-size: 12px; color: #303d57; font-weight: 600; }
            QLabel#connection { font-size: 12px; font-weight: 700; color: #5145a4; }
            QLabel#health { font-size: 13px; font-weight: 700; color: #a33848; }
            QLabel#owner { font-size: 12px; color: #3935a5; font-weight: 500; }
            QPushButton {
                background: #f0eef8; color: #3935a5; border: 1px solid #d5d2ec;
                border-radius: 8px; padding: 6px 12px; font-weight: 650;
            }
            QPushButton:hover { background: #e5e3f5; border-color: #3935a5; }
            QPushButton:focus { border: 2px solid #5145a4; }
            QPushButton:disabled { color: #929bab; background: #f1f3f6; border-color: #e2e6ec; }
            QComboBox {
                background: #ffffff; color: #272641; border: 1px solid #d5d2ec;
                border-radius: 8px; padding: 5px 9px;
            }
            QComboBox:disabled { color: #929bab; background: #f1f3f6; }
            QComboBox::drop-down { border: none; width: 24px; }
        """)
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.window.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        outer.setContentsMargins(18, 12, 18, 12)
        outer.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        brand = QtWidgets.QVBoxLayout()
        brand.setSpacing(1)
        brand.addWidget(self._label("ESTACIÓN COOPERATIVA  /  LABORATORIO", "eyebrow"))
        brand.addWidget(self._label("JACO2  ·  Estación base", "brand"))
        header.addLayout(brand)
        header.addStretch()
        self.connection = self._label("Conectando…", "connection")
        self.connection.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        header.addWidget(self.connection)
        outer.addLayout(header)

        content = QtWidgets.QHBoxLayout()
        content.setSpacing(12)
        outer.addLayout(content, 1)
        self.view = RobotView()
        content.addWidget(self.view, 1)

        # Reflow cards to use the height available without scrolling.
        sidebar = QtWidgets.QWidget()
        sidebar.setObjectName("sidebar")
        self.sidebar = sidebar
        sidebar.setMinimumWidth(440)
        sidebar.setMaximumWidth(490)
        content.addWidget(sidebar)
        sidebar_grid = QtWidgets.QHBoxLayout(sidebar)
        self.sidebar_grid = sidebar_grid
        sidebar_grid.setContentsMargins(0, 0, 0, 0)
        sidebar_grid.setSpacing(9)
        controls_column = QtWidgets.QVBoxLayout()
        info_column = QtWidgets.QVBoxLayout()
        self.controls_column, self.info_column = controls_column, info_column
        controls_column.setSpacing(9)
        info_column.setSpacing(9)
        sidebar_grid.addLayout(controls_column, 1)
        sidebar_grid.addLayout(info_column, 1)

        status_card, status_layout = self._card()
        self.status_title = self._label("ESTADO DEL ROBOT", "eyebrow")
        status_layout.addWidget(self.status_title)
        status_row = QtWidgets.QHBoxLayout()
        self.room = self._label("Sala —", "section")
        self.room.setWordWrap(True)
        self.health = self._label("Sin telemetría", "health")
        self._health_status = None
        self.health.setWordWrap(True)
        status_row.addWidget(self.room, 1)
        status_row.addWidget(self.health)
        status_layout.addLayout(status_row)
        controls_column.addWidget(status_card)
        if self.output_control is not None:
            output_card, output_layout = self._card()
            output_layout.addWidget(self._label("CONTROL DEL BRAZO", "eyebrow"))
            self.output_status = self._label("Esperando estado…", "section")
            self.output_status.setWordWrap(True)
            output_layout.addWidget(self.output_status)
            self.output_button = QtWidgets.QPushButton("Esperando estado…")
            self.output_button.setEnabled(False)
            self.output_button.clicked.connect(self.toggle_output)
            output_layout.addWidget(self.output_button)
            self.output_detail = self._label("El armado se realiza en esta estación.", "caption")
            self.output_detail.setWordWrap(True)
            output_layout.addWidget(self.output_detail)
            if self.physical:
                self.usb_status = self._label("Esperando estado USB…", "caption")
                self.usb_status.setWordWrap(True)
                output_layout.addWidget(self.usb_status)
                usb_buttons = QtWidgets.QHBoxLayout()
                self.reconnect_button = QtWidgets.QPushButton("Detectar USB")
                self.reconnect_button.setToolTip("Detectar y conectar el brazo por USB")
                self.reconnect_button.setEnabled(False)
                self.reconnect_button.clicked.connect(self.output_control.reconnect)
                usb_buttons.addWidget(self.reconnect_button)
                self.disconnect_button = QtWidgets.QPushButton("Desconectar USB")
                self.disconnect_button.setToolTip("Desconectar el brazo de forma segura")
                self.disconnect_button.setEnabled(False)
                self.disconnect_button.clicked.connect(self.output_control.disconnect)
                usb_buttons.addWidget(self.disconnect_button)
                output_layout.addLayout(usb_buttons)
                self.recovery_status = self._label("Recuperación desactivada", "caption")
                self.recovery_status.setWordWrap(True)
                output_layout.addWidget(self.recovery_status)
                recovery_buttons = QtWidgets.QHBoxLayout()
                self.recovery_button = QtWidgets.QPushButton("Activar recuperación")
                self.recovery_button.setToolTip("Activar o desactivar la recuperación por articulaciones")
                self.recovery_button.clicked.connect(self.toggle_recovery)
                recovery_buttons.addWidget(self.recovery_button)
                self.recovery_window_button = QtWidgets.QPushButton("Controles de joints")
                self.recovery_window_button.clicked.connect(self.open_recovery)
                recovery_buttons.addWidget(self.recovery_window_button)
                output_layout.addLayout(recovery_buttons)
            controls_column.addWidget(output_card)

        if self.output_control is not None:
            shutdown_card, shutdown_layout = self._card()
            shutdown_layout.addWidget(self._label("FINALIZAR SESIÓN", "eyebrow"))
            self.shutdown_button = QtWidgets.QPushButton("Cerrar estación de forma segura")
            self.shutdown_button.setStyleSheet("QPushButton { background: #fff0f1; color: #8e2634; border: 1px solid #d9a9ae; } QPushButton:disabled { background: #eef0f3; color: #737d8b; border-color: #d6dbe2; }")
            self.shutdown_button.clicked.connect(self.shutdown_station)
            shutdown_layout.addWidget(self.shutdown_button)
            self.shutdown_detail = self._label("Detiene la salida y cierra el brazo, la web y ROS 2 de esta estación.", "caption")
            self.shutdown_detail.setWordWrap(True)
            shutdown_layout.addWidget(self.shutdown_detail)
            controls_column.addWidget(shutdown_card)

        task_card, task_layout = self._card()
        task_layout.addWidget(self._label("TAREA COOPERATIVA · VIRTUAL", "eyebrow"))
        self.task_status = self._label("Esperando juego…", "section")
        self.task_status.setWordWrap(True); task_layout.addWidget(self.task_status)
        self.task_detail = self._label("Lleva cada muñeco al círculo de su color.", "caption")
        self.task_detail.setWordWrap(True); task_layout.addWidget(self.task_detail)
        if self.output_control is not None:
            self.task_object = QtWidgets.QComboBox()
            for spec in OBJECTS:
                self.task_object.addItem(spec["name"])
            task_row = QtWidgets.QHBoxLayout()
            task_row.addWidget(self.task_object, 1)
            self.task_reset_one = QtWidgets.QPushButton("Reiniciar muñeco")
            self.task_reset_one.setToolTip("Devolver el muñeco seleccionado a su posición inicial")
            self.task_reset_one.clicked.connect(self.reset_selected_object)
            task_row.addWidget(self.task_reset_one)
            task_layout.addLayout(task_row)
            self.task_reset = QtWidgets.QPushButton("Reiniciar objetos virtuales")
            self.task_reset.clicked.connect(self.output_control.reset_game)
            task_layout.addWidget(self.task_reset)
        info_column.addWidget(task_card)

        telemetry_card, telemetry_layout = self._card()
        self.telemetry_title = self._label("TELEMETRÍA EN VIVO", "eyebrow")
        telemetry_layout.addWidget(self.telemetry_title)
        metrics = QtWidgets.QGridLayout()
        metrics.setHorizontalSpacing(8)
        metrics.setVerticalSpacing(6)
        self.fields = {}
        for row, (key, title) in enumerate((
            ("position", "Posición (m)"),
            ("quaternion", "Orientación"),
            ("linear", "Vel. lineal (m/s)"),
            ("angular", "Vel. angular (rad/s)"),
            ("joints", "Joints (rad)"),
            ("fingers", "Dedos sim. (rad)"),
        )):
            label = self._label(title, "caption")
            value = self._label("—", "value")
            value.setWordWrap(True)
            metrics.addWidget(label, row, 0)
            metrics.addWidget(value, row, 1)
            self.fields[key] = value
        telemetry_layout.addLayout(metrics)
        info_column.addWidget(telemetry_card)

        axes_card, axes_layout = self._card()
        axes_layout.addWidget(self._label("TRABAJO COOPERATIVO", "eyebrow"))
        axes_layout.addWidget(self._label("Componentes y propietarios", "section"))
        axes_grid = QtWidgets.QGridLayout()
        axes_grid.setHorizontalSpacing(12)
        axes_grid.setVerticalSpacing(7)
        self.owners = {}
        for index, axis in enumerate(("X", "Y", "Z", "ROLL", "PITCH", "YAW", "PINZA")):
            label = self._label(f"{axis}  ·  Disponible", "owner")
            label.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
            axes_grid.addWidget(label, index // 2, index % 2)
            self.owners[axis] = label
        axes_layout.addLayout(axes_grid)
        info_column.addWidget(axes_card)

        event_card, event_layout = self._card()
        event_layout.addWidget(self._label("ÚLTIMO EVENTO", "eyebrow"))
        self.event = self._label("Sin eventos recientes", "caption")
        self.event.setWordWrap(True)
        event_layout.addWidget(self.event)
        info_column.addWidget(event_card)
        self._main_cards = [status_card]
        if self.output_control is not None:
            self._main_cards.append(output_card)
        self._main_cards += [task_card, telemetry_card, axes_card, event_card]
        if self.output_control is not None:
            self._main_cards.append(shutdown_card)
        self._wide_cards = [task_card, telemetry_card, event_card]
        if self.output_control is not None:
            self._wide_cards.append(shutdown_card)
        self._single_column = None
        self._reflow_cards(1080)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        for key in (QtCore.Qt.Key_F11, QtCore.Qt.Key_Escape):
            shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self.window)
            shortcut.activated.connect(self.toggle_fullscreen)
        self.window.closeEvent = self.close_event
        self.window.resizeEvent = self.resize_event

    def _reflow_cards(self, height):
        single = height >= 950
        if single == self._single_column:
            return
        self._single_column = single
        for card in self._main_cards:
            self.controls_column.removeWidget(card)
            self.info_column.removeWidget(card)
        if single:
            self.sidebar.setMinimumWidth(440)
            self.sidebar.setMaximumWidth(490)
            for card in self._main_cards:
                self.controls_column.addWidget(card)
            self.sidebar_grid.removeItem(self.info_column)
        else:
            self.sidebar.setMaximumWidth(760)
            self.sidebar.setMinimumWidth(660)
            self.sidebar_grid.addLayout(self.info_column, 1)
            for card in self._main_cards:
                (self.info_column if card in self._wide_cards else self.controls_column).addWidget(card)

    def resize_event(self, event):
        self._reflow_cards(event.size().height())
        QtWidgets.QMainWindow.resizeEvent(self.window, event)

    @staticmethod
    def _label(text, name):
        label = QtWidgets.QLabel(text)
        label.setObjectName(name)
        return label

    @staticmethod
    def _card():
        frame = QtWidgets.QFrame()
        frame.setObjectName("card")
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(12, 7, 12, 7)
        layout.setSpacing(4)
        return frame, layout

    def toggle_fullscreen(self):
        self.window.showNormal() if self.window.isFullScreen() else self.window.showFullScreen()

    def run(self):
        self.observer.start()
        self.window.showFullScreen()
        self.timer.start(50)
        self.refresh()
        self.app.exec_()

    def toggle_output(self):
        if self.output_control is None:
            return
        enabled = self.output_control.poll()
        if enabled is None or (not enabled and not self.state.snapshot().connected):
            return
        if not enabled:
            answer = QtWidgets.QMessageBox.question(
                self.window, "Armar salida", "¿Armar la salida del bridge directo?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                return
        self.output_control.request(not enabled)

    def reset_selected_object(self):
        index = self.task_object.currentIndex()
        task = self.state.snapshot().virtual_task
        if task is None:
            return
        if any(item['id'] != index and math.dist(item['position'], INITIAL[index]) < .09 for item in task['objects']):
            QtWidgets.QMessageBox.information(self, "Posición ocupada", "Otro muñeco ocupa la posición inicial. Retíralo antes de reiniciar este muñeco.")
            return
        self.output_control.reset_object(index)

    def toggle_recovery(self):
        control = self.output_control
        active = control.recovery_state()
        if active is None:
            return
        if self.recovery_dialog is not None:
            self.recovery_dialog.stop()
        control.set_recovery(not active)

    def open_recovery(self):
        if self.recovery_dialog is None:
            self.recovery_dialog = RecoveryDialog(self.window, self.output_control)
        self.recovery_dialog.show()
        self.recovery_dialog.raise_()
        self.recovery_dialog.activateWindow()

    def shutdown_station(self):
        if self.output_control is None or self.output_control.shutdown_future is not None:
            return
        answer = QtWidgets.QMessageBox.question(
            self.window, "Cerrar estación", "¿Detener el brazo y cerrar la estación, la web y sus procesos ROS 2?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        if self.recovery_dialog is not None:
            self.recovery_dialog.stop()
        if self.output_control.prepare_shutdown():
            self.shutdown_locked = True
            self.shutdown_started = time.monotonic()
            self.shutdown_button.setEnabled(False)
        self.shutdown_detail.setText(self.output_control.shutdown_message)

    def _finish_shutdown_if_ready(self):
        control = self.output_control
        if not self.shutdown_locked:
            return
        if control.shutdown_result is True:
            # Avoid retrying the signal if the launcher is already shutting down.
            control.shutdown_result = None
            try:
                from .station_shutdown import signal_station_launcher
                signal_station_launcher()
                self.shutdown_detail.setText("Cerrando los servicios de la estación…")
            except RuntimeError as error:
                self.shutdown_detail.setText(str(error))
                self.shutdown_button.setEnabled(True)
            return
        if control.shutdown_result is False:
            control.shutdown_result = None
            self.shutdown_detail.setText(control.shutdown_message)
            self.shutdown_button.setEnabled(True)
        elif control.shutdown_future is not None and time.monotonic() - self.shutdown_started > 8:
            self.shutdown_detail.setText("Esperando confirmación del bridge. No retire el cable todavía.")

    def close_event(self, event):
        if self.recovery_dialog is not None:
            self.recovery_dialog.stop()
        self.timer.stop()
        if self.output_control is not None:
            self.output_control.close()
        self.observer.stop()
        self.view.widget.GetRenderWindow().Finalize()
        event.accept()

    @staticmethod
    def format_vector(value):
        return "—" if value is None else "[" + ", ".join(f"{v:.4f}" for v in value) + "]"

    def refresh(self):
        snap = self.state.snapshot()
        if self.output_control is not None:
            enabled = self.output_control.poll()
            self._finish_shutdown_if_ready()
            self.output_status.setText("Estado desconocido" if enabled is None else "Salida armada" if enabled else "Salida desarmada")
            self.output_status.setStyleSheet("color: " + ("#8a5c16" if enabled is None else "#a4343f" if enabled else "#257251"))
            color = "#b32332" if enabled else "#f4cdd0" if enabled is False else "#e5e7eb"
            foreground = "#ffffff" if enabled else "#64202a" if enabled is False else "#555e6e"
            self.output_button.setStyleSheet(f"QPushButton {{ background: {color}; color: {foreground}; border: 1px solid #b65a64; }} QPushButton:disabled {{ background: #e8e8ed; color: #666d7a; border-color: #cbd0d8; }}")
            self.output_button.setText("Esperando estado…" if enabled is None else "Desarmar salida" if enabled else "Armar salida")
            self.output_button.setEnabled(enabled is not None and self.output_control.future is None and (enabled or snap.connected))
            if self.physical:
                usb = self.output_control.usb_state()
                self.usb_status.setText("USB conectado" if usb is True else "USB desconectado" if usb is False else "Estado USB no disponible")
                self.reconnect_button.setEnabled(usb is False and self.output_control.reconnect_future is None and self.output_control.disconnect_future is None and time.monotonic() - self.output_control.reconnect_at >= 1.0)
                self.disconnect_button.setEnabled(usb is True and self.output_control.disconnect_future is None and self.output_control.reconnect_future is None and self.output_control.future is None)
                if self.output_control.disconnect_future is not None:
                    self.output_button.setEnabled(False)
                if self.output_control.reconnect_message:
                    self.usb_status.setText(self.usb_status.text() + " · " + self.output_control.reconnect_message)
                if usb is not True and not enabled:
                    self.output_button.setEnabled(False)
            if self.physical:
                active = self.output_control.recovery_state()
                if active and snap.connected and snap.room_state != "FAULT":
                    self.output_control.heartbeat()
                available = active is not None and self.output_control.recovery_future is None and snap.connected
                can_enter = enabled is True and usb is True and snap.room_state in ("READY", "ACTIVE")
                returning = self.output_control.recovery_returning
                self.recovery_button.setEnabled(available and not returning and (active or can_enter))
                self.recovery_button.setText("Restableciendo web…" if returning else "Desactivar recuperación" if active else "Activar recuperación")
                self.recovery_window_button.setEnabled(active is True and not returning and snap.room_state == "PAUSED" and enabled is True)
                mode_text = "Recuperación activa · web pausada" if active else "Recuperación desactivada" if active is False else "Esperando estado de recuperación"
                self.recovery_status.setText(mode_text + (" · " + self.output_control.recovery_message if self.output_control.recovery_message else ""))
                if self.recovery_dialog is not None:
                    age = None if snap.updated_monotonic is None else (snap.telemetry_age_sec or 0) + max(0, time.monotonic() - snap.updated_monotonic)
                    live = snap.connected and age is not None and age <= STALE_SEC and snap.joint_positions is not None
                    self.recovery_dialog.tick(active is True and not returning and enabled is True and snap.room_state == "PAUSED" and live, snap.joint_positions if live else None)
            result_messages = {"enabled": "Salida armada correctamente.", "disabled": "Salida desarmada correctamente.", "unchanged": "El estado de la salida no cambió."}
            detail = result_messages.get(self.output_control.message, self.output_control.message) or "El armado se realiza en esta estación."
            if snap.room_state == "FAULT":
                detail += " La sala está en fallo; requiere rearme de administrador."
            self.output_detail.setText(detail)
        if self.output_control is not None and self.shutdown_locked:
            self.output_button.setEnabled(False)
            if self.physical:
                for button in (self.reconnect_button, self.disconnect_button, self.recovery_button, self.recovery_window_button):
                    button.setEnabled(False)
                if self.recovery_dialog is not None:
                    self.recovery_dialog.tick(False, None)
        self.connection.setText(snap.connection_label)
        room_labels = {"NOT_READY": "Esperando armado", "READY": "Lista", "ACTIVE": "En operación", "FAULT": "Fallo", "PAUSED": "Web pausada", "SIM_ONLY": "Solo pinza simulada"}
        self.room.setText(f"Sala {snap.room_id}\n{room_labels.get(snap.room_state, snap.room_state)}")
        age = None if snap.updated_monotonic is None else (snap.telemetry_age_sec or 0) + max(0, time.monotonic() - snap.updated_monotonic)
        fresh = snap.connected and age is not None and age <= STALE_SEC and snap.telemetry_source in ("ros_gazebo_jaco2_fk", "ros_physical_jaco2_fk", "ros_mock_jaco2_fk") and snap.joint_positions is not None
        if snap.telemetry_source == "ros_physical_jaco2_fk":
            self.window.setWindowTitle("JACO2 · Estación base · Brazo físico")
            self.status_title.setText("ESTADO DEL BRAZO FÍSICO")
            self.telemetry_title.setText("BRAZO FÍSICO (USB)")
        elif snap.telemetry_source == "ros_gazebo_jaco2_fk":
            self.window.setWindowTitle("JACO2 · Estación base · Simulación")
            self.status_title.setText("ESTADO DE LA SIMULACIÓN")
            self.telemetry_title.setText("BRAZO SIMULADO")
        elif snap.telemetry_source == "ros_mock_jaco2_fk":
            self.window.setWindowTitle("JACO2 · Estación base · Simulación")
            self.status_title.setText("ESTADO DE LA SIMULACIÓN")
            self.telemetry_title.setText("BRIDGE SIMULADO")
        health_status = "fault" if snap.room_state == "FAULT" else "live" if fresh else "stale"
        if health_status != self._health_status:
            self.health.setStyleSheet("color: " + {"fault": "#a33848", "live": "#23724f", "stale": "#d97706"}[health_status])
            self._health_status = health_status
        if health_status == "fault":
            self.health.setText("FALLO · movimiento bloqueado")
        elif health_status == "live":
            self.health.setText(f"Telemetría en vivo\n{age:.2f} s")
        else:
            self.health.setText("Telemetría no disponible")
        for name, value in (("position", snap.position), ("quaternion", snap.quaternion),
                            ("linear", snap.linear), ("angular", snap.angular), ("joints", snap.joint_positions), ("fingers", snap.finger_positions)):
            self.fields[name].setText(self.format_vector(value if fresh else None))
        for axis, label in self.owners.items():
            text = f"{axis}  ·  {snap.owners.get(axis, 'Disponible')}"
            label.setToolTip(text)
            label.setText(label.fontMetrics().elidedText(text, QtCore.Qt.ElideRight, max(label.width(), 20)))
        self.event.setText(snap.events[0] if snap.events else "Sin eventos recientes")
        task = snap.virtual_task if snap.connected else None
        self.task_status.setText(f"Muñecos entregados: {task['delivered']} / 5" if task is not None else "Juego virtual no disponible")
        if task is not None and task['held'] is not None:
            self.task_detail.setText(f"Sujetando: {OBJECTS[task['held']]['name']} · llévalo a su círculo.")
        elif task is not None and task.get("release_reason") == "impacto":
            self.task_detail.setText("Muñeco soltado por presión. Abre la pinza antes de sujetarlo de nuevo.")
        else:
            self.task_detail.setText("Lleva cada muñeco a su círculo. El agarre es virtual; los contactos no detienen el brazo físico." if self.physical else "Lleva cada muñeco al círculo de su color. El agarre es virtual y asistido.")
        if self.output_control is not None:
            self.task_reset.setEnabled(task is not None and not self.shutdown_locked)
            self.task_reset_one.setEnabled(task is not None and not self.shutdown_locked)
            self.task_object.setEnabled(task is not None and not self.shutdown_locked)
        self.view.update_task(task)
        self.view.update_joints(snap.joint_positions if fresh else None, snap.finger_positions if fresh else None, snap.finger_source if fresh else None)
