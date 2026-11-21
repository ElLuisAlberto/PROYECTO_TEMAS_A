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
FINGER_ORIGINS = (
    ((0.00279, 0.03126, -0.11467), (-1.7047873384941834, 0.6476144647144773, 1.67317415161155)),
    ((0.02226, -0.02707, -0.11482), (-1.570796327, 0.649262481663582, -1.38614049188413)),
    ((-0.02226, -0.02707, -0.11482), (-1.570796327, 0.649262481663582, -1.75545216211587)),
)


def link_transforms(joints):
    """World transforms for base and six links, following the Jaco2 Xacro."""
    values = kin._validated_joints(joints)
    result = [np.eye(4)]
    transform = np.eye(4)
    for value, (translation, rpy) in zip(values, kin._JOINT_ORIGINS):
        origin = np.eye(4)
        origin[:3, :3] = kin._rpy_rotation(rpy)
        origin[:3, 3] = translation
        rotation = np.eye(4)
        rotation[:3, :3] = kin._z_rotation(value)
        transform = transform @ origin @ rotation
        result.append(transform.copy())
    return result


def _origin_transform(translation, rpy):
    result = np.eye(4)
    result[:3, :3] = kin._rpy_rotation(rpy)
    result[:3, 3] = translation
    return result


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

        def add_mesh(index, name, offset=None):
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
                self._robot_parts.append((actor, index, np.eye(4) if offset is None else offset))

        for index, name in enumerate(MESHES):
            add_mesh(index, name)
        for index in (1, 2, 3):
            add_mesh(index, "ring_big")
        for index in (4, 5, 6):
            add_mesh(index, "ring_small")
        for translation, rpy in FINGER_ORIGINS:
            finger = _origin_transform(translation, rpy)
            add_mesh(6, "finger_proximal", finger)
            add_mesh(6, "finger_distal", finger @ _origin_transform((0.044, -0.003, 0.0), (0.0, 0.0, 0.0)))

        self.reset_view(render=False)
        self.widget.Initialize()

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

    def update_joints(self, joints):
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
            for actor, index, offset in self._robot_parts:
                transform = transforms[index] @ offset
                matrix = vtk.vtkMatrix4x4()
                for row in range(4):
                    for column in range(4):
                        matrix.SetElement(row, column, float(transform[row, column]))
                actor.SetUserMatrix(matrix)
                actor.SetVisibility(True)
            self.status.setText("EN VIVO · Vista reconstruida · Dedos fijos")
        self.renderer.ResetCameraClippingRange()
        self.widget.GetRenderWindow().Render()


class QtDesktopConsole:
    """Fullscreen observer UI with an optional local direct-output switch."""

    def __init__(self, state, observer, direct_bridge=False):
        self.state, self.observer = state, observer
        self.output_control = None
        if direct_bridge:
            from .direct_output_control import DirectOutputControl
            self.output_control = DirectOutputControl()
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("JACO2 · Estación base")
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
            QScrollArea { border: none; background: transparent; }
            QScrollBar:vertical { background: transparent; width: 8px; margin: 2px; }
            QScrollBar::handle:vertical { background: #c7cfdb; border-radius: 3px; min-height: 28px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        """)
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.window.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
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

        sidebar_scroll = QtWidgets.QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        sidebar_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setMinimumWidth(370)
        sidebar_scroll.setMaximumWidth(450)
        content.addWidget(sidebar_scroll)
        sidebar = QtWidgets.QWidget()
        sidebar.setObjectName("sidebar")
        sidebar_scroll.setWidget(sidebar)
        sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.setSpacing(9)

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
        sidebar_layout.addWidget(status_card)
        if self.output_control is not None:
            output_card, output_layout = self._card()
            output_layout.addWidget(self._label("SALIDA DEL BRIDGE DIRECTO", "eyebrow"))
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
            sidebar_layout.addWidget(output_card)

        telemetry_card, telemetry_layout = self._card()
        self.telemetry_title = self._label("TELEMETRÍA EN VIVO", "eyebrow")
        telemetry_layout.addWidget(self.telemetry_title)
        telemetry_layout.addWidget(self._label("Estado en tiempo real", "section"))
        metrics = QtWidgets.QGridLayout()
        metrics.setHorizontalSpacing(12)
        metrics.setVerticalSpacing(5)
        self.fields = {}
        for row, (key, title) in enumerate((
            ("position", "Posición world (m)"),
            ("quaternion", "Orientación xyzw"),
            ("linear", "Velocidad lineal comandada (m/s)"),
            ("angular", "Velocidad angular comandada (rad/s)"),
            ("joints", "Articulaciones (rad)"),
        )):
            label = self._label(title, "caption")
            value = self._label("—", "value")
            value.setWordWrap(True)
            metrics.addWidget(label, row * 2, 0)
            metrics.addWidget(value, row * 2 + 1, 0)
            self.fields[key] = value
        telemetry_layout.addLayout(metrics)
        sidebar_layout.addWidget(telemetry_card)

        axes_card, axes_layout = self._card()
        axes_layout.addWidget(self._label("TRABAJO COOPERATIVO", "eyebrow"))
        axes_layout.addWidget(self._label("Componentes y propietarios", "section"))
        axes_grid = QtWidgets.QGridLayout()
        axes_grid.setHorizontalSpacing(12)
        axes_grid.setVerticalSpacing(7)
        self.owners = {}
        for index, axis in enumerate(("X", "Y", "Z", "ROLL", "PITCH", "YAW")):
            label = self._label(f"{axis}  ·  Disponible", "owner")
            label.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
            axes_grid.addWidget(label, index // 2, index % 2)
            self.owners[axis] = label
        axes_layout.addLayout(axes_grid)
        sidebar_layout.addWidget(axes_card)

        event_card, event_layout = self._card()
        event_layout.addWidget(self._label("ÚLTIMO EVENTO", "eyebrow"))
        self.event = self._label("Sin eventos recientes", "caption")
        self.event.setWordWrap(True)
        event_layout.addWidget(self.event)
        sidebar_layout.addWidget(event_card)
        sidebar_layout.addStretch(1)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        for key in (QtCore.Qt.Key_F11, QtCore.Qt.Key_Escape):
            shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self.window)
            shortcut.activated.connect(self.toggle_fullscreen)
        self.window.closeEvent = self.close_event

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
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(5)
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

    def close_event(self, event):
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
            self.output_status.setText("Estado desconocido" if enabled is None else "Salida armada" if enabled else "Salida desarmada")
            self.output_status.setStyleSheet("color: " + ("#8a5c16" if enabled is None else "#a4343f" if enabled else "#257251"))
            self.output_button.setText("Esperando estado…" if enabled is None else "Desarmar salida" if enabled else "Armar salida")
            self.output_button.setEnabled(enabled is not None and self.output_control.future is None and (enabled or snap.connected))
            result_messages = {"enabled": "Salida armada correctamente.", "disabled": "Salida desarmada correctamente.", "unchanged": "El estado de la salida no cambió."}
            detail = result_messages.get(self.output_control.message, self.output_control.message) or "El armado se realiza en esta estación."
            if snap.room_state == "FAULT":
                detail += " La sala está en fallo; requiere rearme de administrador."
            self.output_detail.setText(detail)
        self.connection.setText(snap.connection_label)
        room_labels = {"NOT_READY": "Esperando armado", "READY": "Lista", "ACTIVE": "En operación", "FAULT": "Fallo"}
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
                            ("linear", snap.linear), ("angular", snap.angular), ("joints", snap.joint_positions)):
            self.fields[name].setText(self.format_vector(value if fresh else None))
        for axis, label in self.owners.items():
            text = f"{axis}  ·  {snap.owners.get(axis, 'Disponible')}"
            label.setToolTip(text)
            label.setText(label.fontMetrics().elidedText(text, QtCore.Qt.ElideRight, max(label.width(), 20)))
        self.event.setText(snap.events[0] if snap.events else "Sin eventos recientes")
        self.view.update_joints(snap.joint_positions if fresh else None)
