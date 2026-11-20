"""Qt base-station viewer: Jaco2 link meshes driven only by gateway telemetry.

This reconstructs arm posture, not a camera image of the Gazebo world.
"""
from __future__ import annotations

import math
from pathlib import Path
import time

import collada
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

from . import jaco2_kinematics as kin
from .cad_camera import CadCameraStyle, CadVTKWidget

MESH_DIR = (Path(__file__).resolve().parents[2] / "ros2_ws/src/course_teleop_description"
            / "meshes/kinova_j2n6s300")
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
        self.renderer.SetBackground(0.83, 0.79, 0.72)
        self.renderer.SetBackground2(0.97, 0.95, 0.90)
        self.renderer.GradientBackgroundOn()
        self.widget.GetRenderWindow().AddRenderer(self.renderer)
        self.camera_style = CadCameraStyle(self.renderer, self.widget.devicePixelRatioF)
        self.widget.SetInteractorStyle(self.camera_style)
        self.renderer.SetAmbient(0.46, 0.44, 0.40)
        light = vtk.vtkLight()
        light.SetLightTypeToSceneLight()
        light.SetPosition(-2, -3, 5)
        light.SetFocalPoint(0.2, 0, 0.4)
        light.SetColor(1.0, 0.94, 0.83)
        light.SetIntensity(0.9)
        self.renderer.AddLight(light)
        fill = vtk.vtkLight()
        fill.SetLightTypeToSceneLight()
        fill.SetPosition(2, 2, 3)
        fill.SetFocalPoint(0, 0, 0.5)
        fill.SetColor(0.90, 0.94, 1.0)
        fill.SetIntensity(0.45)
        self.renderer.AddLight(fill)

        # These dimensions and poses match the Jaco lab SDF: tabletop top at
        # z=0, floor at z=-0.72. They are not a separate collision model.
        self._box(LAB_TABLE_CENTER, LAB_TABLE_SIZE, (0.66, 0.48, 0.33))
        for center in LAB_LEG_CENTERS:
            self._box(center, LAB_LEG_SIZE, (0.42, 0.39, 0.35))
        self._box((0.1, 0.0, LAB_FLOOR_TOP - 0.01), (5.0, 5.0, 0.02), (0.76, 0.73, 0.68))

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

    def _box(self, center, dimensions, color):
        cube = vtk.vtkCubeSource()
        cube.SetCenter(*center)
        cube.SetXLength(dimensions[0]); cube.SetYLength(dimensions[1]); cube.SetZLength(dimensions[2])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetAmbient(0.35)
        self.renderer.AddActor(actor)

    def reset_view(self, _checked=False, *, render=True):
        self.camera_style.cancel_drag()
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(1.8, -2.4, 1.65)
        camera.SetFocalPoint(0.22, 0.0, 0.25)
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(0.95)
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
    """Fullscreen observer UI. No control or admin messages are exposed."""

    def __init__(self, state, observer):
        self.state, self.observer = state, observer
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("JACO2 · Estación base · Simulación")
        self.window.setStyleSheet("""
            QMainWindow, QWidget#root { background: #f4f0e9; }
            QWidget { color: #27383a; font-family: "Sans"; font-size: 13px; }
            QLabel { background: transparent; border: none; }
            QFrame#viewer, QFrame#card {
                background: #fffdfa; border: 1px solid #d9d1c5; border-radius: 12px;
            }
            QLabel#brand { font-size: 22px; font-weight: 700; color: #27423d; }
            QLabel#eyebrow { font-size: 10px; font-weight: 700; letter-spacing: 1px; color: #6b8277; }
            QLabel#section { font-size: 15px; font-weight: 700; color: #2c413d; }
            QLabel#caption { font-size: 11px; color: #647974; }
            QLabel#value { font-family: "Monospace"; font-size: 12px; color: #253d38; }
            QLabel#connection { font-size: 12px; font-weight: 700; color: #49635b; }
            QLabel#health { font-size: 13px; font-weight: 700; color: #8e4d3d; }
            QLabel#owner { font-size: 12px; color: #415a53; }
            QPushButton {
                background: #f1ebe1; color: #35554b; border: 1px solid #d4c8b9;
                border-radius: 7px; padding: 5px 10px; font-weight: 600;
            }
            QPushButton:hover { background: #e7dfd1; }
            QPushButton:focus { border: 2px solid #35776b; }
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

        sidebar = QtWidgets.QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setMinimumWidth(340)
        sidebar.setMaximumWidth(460)
        sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_layout.setSpacing(9)
        content.addWidget(sidebar)

        status_card, status_layout = self._card()
        status_layout.addWidget(self._label("ESTADO DE LA SIMULACIÓN", "eyebrow"))
        status_row = QtWidgets.QHBoxLayout()
        self.room = self._label("Sala —", "section")
        self.health = self._label("Sin telemetría", "health")
        self._health_status = None
        self.health.setWordWrap(True)
        status_row.addWidget(self.room, 1)
        status_row.addWidget(self.health)
        status_layout.addLayout(status_row)
        sidebar_layout.addWidget(status_card)

        telemetry_card, telemetry_layout = self._card()
        telemetry_layout.addWidget(self._label("BRAZO SIMULADO", "eyebrow"))
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

    def close_event(self, event):
        self.timer.stop()
        self.observer.stop()
        self.view.widget.GetRenderWindow().Finalize()
        event.accept()

    @staticmethod
    def format_vector(value):
        return "—" if value is None else "[" + ", ".join(f"{v:.4f}" for v in value) + "]"

    def refresh(self):
        snap = self.state.snapshot()
        self.connection.setText(snap.connection_label)
        self.room.setText(f"Sala {snap.room_id} · {snap.room_state}")
        age = None if snap.updated_monotonic is None else (snap.telemetry_age_sec or 0) + max(0, time.monotonic() - snap.updated_monotonic)
        fresh = snap.connected and age is not None and age <= STALE_SEC and snap.telemetry_source == "ros_gazebo_jaco2_fk" and snap.joint_positions is not None
        health_status = "fault" if snap.room_state == "FAULT" else "live" if fresh else "stale"
        if health_status != self._health_status:
            self.health.setStyleSheet("color: " + {"fault": "#9b3030", "live": "#276a52", "stale": "#8e4d3d"}[health_status])
            self._health_status = health_status
        if health_status == "fault":
            self.health.setText("FALLO · movimiento bloqueado")
        elif health_status == "live":
            self.health.setText(f"En vivo · {age:.2f} s")
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
