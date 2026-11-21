"""Presentation-only workspace for the existing supervised JACO controls.

All operational widgets are moved, never copied.  Their signals, timers and
ROS ownership remain with JointControlWindow.
"""

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QLayout, QListWidget, QPushButton,
    QScrollArea, QShortcut, QStackedWidget, QTabWidget, QVBoxLayout, QWidget,
)


WARM_STYLESHEET = """
QMainWindow, QWidget#workspaceShell { background: #f3eee6; color: #302b26; }
QWidget { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; font-size: 13px; }
QFrame#workspaceHeader { background: #fffaf3; border-bottom: 1px solid #d8cbb9; }
QFrame#workspaceNavigation { background: #e8dfd1; border-right: 1px solid #d8cbb9; }
QLabel#workspaceBrand { color: #704126; font-size: 22px; font-weight: bold; }
QLabel#workspaceSubtitle { color: #665b4e; font-size: 12px; }
QLabel#workspacePageTitle { color: #4b3526; font-size: 20px; font-weight: bold; }
QListWidget#workspaceNavigationList { background: transparent; border: 0; outline: 0; }
QListWidget#workspaceNavigationList::item { padding: 13px 10px; margin: 3px 0;
    border-radius: 6px; color: #493e33; }
QListWidget#workspaceNavigationList::item:selected { background: #805035; color: #fffaf3; }
QListWidget#workspaceNavigationList::item:hover { background: #d9c9b6; color: #302b26; }
QGroupBox { background: #fffaf3; color: #493e33; font-weight: bold;
    border: 1px solid #d8cbb9; border-radius: 8px; margin-top: 12px; padding: 14px 10px 10px; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QPushButton { background: #e9dece; color: #392e25; border: 1px solid #cdbba5;
    border-radius: 6px; padding: 8px 10px; min-height: 18px; }
QPushButton:hover { background: #deccb7; border-color: #a47b58; }
QPushButton:pressed, QPushButton:checked { background: #805035; color: #fffaf3; }
QPushButton:disabled { background: #eee9e1; color: #837b70; border-color: #d9d1c6; }
QPushButton:focus, QDoubleSpinBox:focus, QComboBox:focus, QListWidget:focus {
    border: 2px solid #805035; }
QPushButton#sendButton, QPushButton#cartesianSendButton {
    background: #805035; color: #fffaf3; font-weight: bold; padding: 11px; }
QPushButton#sendButton:disabled, QPushButton#cartesianSendButton:disabled {
    background: #d9d0c3; color: #625b51; }
QDoubleSpinBox, QComboBox { background: #fffcf7; color: #302b26;
    border: 1px solid #bcab94; border-radius: 4px; padding: 5px; min-height: 20px; }
QTableWidget { background: #fffcf7; alternate-background-color: #f1e8dc;
    color: #302b26; gridline-color: #dfd4c5; border: 1px solid #d8cbb9; }
QHeaderView::section { background: #e9dece; color: #493e33; border: 0;
    border-bottom: 1px solid #cdbba5; padding: 7px; font-weight: bold; }
QScrollArea { background: transparent; border: 0; }
QScrollArea > QWidget > QWidget { background: transparent; }
QTabWidget::pane { border: 1px solid #d8cbb9; border-radius: 5px; }
QTabBar::tab { background: #e9dece; color: #493e33; padding: 8px 12px; }
QTabBar::tab:selected { background: #805035; color: #fffaf3; }
QSlider::groove:horizontal { background: #d9cbb9; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #805035; width: 15px; margin: -5px 0;
    border-radius: 6px; }
QLabel#headerLabel { color: #493e33; font-weight: bold; }
QLabel#poseValue { color: #704126; font-weight: bold; font-size: 16px; }
QLabel#predictionState { font-size: 18px; font-weight: bold; padding: 5px; }
QLabel#safetyBanner { background: #f0e5d5; color: #59422f; border: 1px solid #d7c3a7;
    border-radius: 6px; padding: 7px; font-weight: bold; }
QLabel#readinessLabel { border: 1px solid #cfbdA5; border-radius: 5px; padding: 7px;
    font-weight: bold; }
QLabel#statusLabel { background: #fffaf3; color: #493e33; padding: 5px; }
QLabel#connectionLabel { color: #6d4b2f; font-weight: bold; }
"""


def _page(window, page_id, title):
    widget = QWidget()
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(6, 4, 10, 10)
    layout.setSpacing(10)
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(widget)
    window.workspace_page_ids.append(page_id)
    window.workspace_page_layouts[page_id] = layout
    window.workspace_navigation.addItem(title)
    window.workspace_pages.addWidget(scroll)
    return layout


def _put(layout, *widgets):
    for widget in widgets:
        if widget is not None:
            layout.addWidget(widget)


class _ResponsiveColumns(QWidget):
    """Stack secondary controls when the workspace has a narrow viewport."""

    def __init__(self, first, second):
        super().__init__()
        self.first = first
        self.second = second
        self.stacked = None
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        # Do not let the two-column minimum lock the viewport at desktop width.
        self.grid.setSizeConstraint(QLayout.SetNoConstraint)
        self.grid.setColumnStretch(0, 3)
        self.grid.setColumnStretch(1, 2)
        self._reflow(True)

    def minimumSizeHint(self):
        return QSize(480, self.grid.minimumSize().height())

    def _reflow(self, stacked):
        if self.stacked == stacked:
            return
        self.stacked = stacked
        self.grid.removeWidget(self.first)
        self.grid.removeWidget(self.second)
        self.grid.addWidget(self.first, 0, 0, 1, 2 if stacked else 1)
        self.grid.addWidget(self.second, 1 if stacked else 0, 0 if stacked else 1,
                            1, 2 if stacked else 1)
        self.updateGeometry()

    def resizeEvent(self, event):
        self._reflow(event.size().width() < 1350)
        super().resizeEvent(event)


def add_workspace_page(window, page_id, title, widget):
    """Add a read-only/auxiliary page or insert content into an existing page."""
    layout = window.workspace_page_layouts.get(page_id)
    if layout is None:
        layout = _page(window, page_id, title)
        layout.addWidget(widget)
        layout.addStretch()
    else:
        layout.insertWidget(0, widget)
    return layout


def select_workspace_page(window, page_id):
    """Select a task through the same navigation path as a user click."""
    window.workspace_navigation.setCurrentRow(
        window.workspace_page_ids.index(page_id)
    )


class _AuxiliaryWindow(QDialog):
    """A presentation window; closing it restores the existing page widget."""

    def __init__(self, window, page_id, page_widget, placeholder):
        super().__init__(window)
        self.owner = window
        self.page_id = page_id
        self.page_widget = page_widget
        self.placeholder = placeholder
        self.setWindowTitle(
            'JACO2 · ' + window.workspace_navigation.item(
                window.workspace_page_ids.index(page_id)
            ).text()
        )
        self.setStyleSheet(WARM_STYLESHEET)
        self.setModal(False)
        self.resize(1000, 720)
        layout = QVBoxLayout(self)
        layout.addWidget(page_widget)

    def closeEvent(self, event):
        index = self.owner.workspace_page_ids.index(self.page_id)
        self.layout().removeWidget(self.page_widget)
        self.owner.workspace_pages.removeWidget(self.placeholder)
        self.owner.workspace_pages.insertWidget(index, self.page_widget)
        if self.owner.workspace_navigation.currentRow() == index:
            self.owner.workspace_pages.setCurrentIndex(index)
        self.placeholder.deleteLater()
        self.owner.workspace_detached_dialogs.pop(self.page_id, None)
        super().closeEvent(event)
        self.deleteLater()


def detach_workspace_page(window, page_id):
    """Open diagnostics/history in one auxiliary window using the same widget."""
    if page_id not in ('diagnostics', 'history'):
        raise ValueError('Only diagnostics and history can be detached')
    existing = window.workspace_detached_dialogs.get(page_id)
    if existing is not None:
        existing.show()
        existing.raise_()
        existing.activateWindow()
        return existing
    if page_id == 'diagnostics':
        for button in (window.jog_mode_button, window.cartesian_jog_mode_button):
            if button is not None and button.isChecked():
                button.setChecked(False)
        dialog = QDialog(window)
        dialog.setWindowTitle('JACO2 · Diagnóstico')
        dialog.setStyleSheet(WARM_STYLESHEET)
        dialog.resize(1000, 720)
        layout = QVBoxLayout(dialog)
        content = window.workspace_diagnostic_content
        layout.addWidget(content)

        def restore():
            window.workspace_diagnostic_container.addWidget(content)
            window.workspace_detached_dialogs.pop('diagnostics', None)
            dialog.deleteLater()

        dialog.finished.connect(restore)
        window.workspace_detached_dialogs['diagnostics'] = dialog
        dialog.show()
        return dialog
    index = window.workspace_page_ids.index(page_id)
    page_widget = window.workspace_pages.widget(index)
    placeholder = QWidget()
    placeholder_layout = QVBoxLayout(placeholder)
    note = QLabel('Esta sección está abierta en una ventana auxiliar.')
    note.setWordWrap(True)
    placeholder_layout.addWidget(note)
    placeholder_layout.addStretch()
    window.workspace_pages.removeWidget(page_widget)
    window.workspace_pages.insertWidget(index, placeholder)
    window.workspace_pages.setCurrentIndex(window.workspace_navigation.currentRow())
    dialog = _AuxiliaryWindow(window, page_id, page_widget, placeholder)
    window.workspace_detached_dialogs[page_id] = dialog
    dialog.show()
    return dialog


def install_workspace(window):
    """Rearrange the already-created controls without creating control owners."""
    groups = window.workspace_groups
    old_central = window.takeCentralWidget()
    # Keep the former container until every operational widget has a new parent.
    window._previous_workspace_container = old_central
    shell = QWidget()
    shell.setObjectName('workspaceShell')
    outer = QVBoxLayout(shell)
    outer.setContentsMargins(0, 0, 0, 0)
    outer.setSpacing(0)

    header = QFrame()
    header.setObjectName('workspaceHeader')
    header_layout = QVBoxLayout(header)
    header_layout.setContentsMargins(18, 12, 18, 10)
    heading = QHBoxLayout()
    brand = QLabel('JACO2 · Control supervisado')
    brand.setObjectName('workspaceBrand')
    heading.addWidget(brand, 1)
    mode = QLabel('BRAZO FÍSICO' if window.is_hardware_mode else 'SIMULACIÓN')
    mode.setObjectName('workspaceSubtitle')
    heading.addWidget(mode)
    header_layout.addLayout(heading)
    _put(header_layout, window.workspace_banner)
    status_row = QHBoxLayout()
    status_row.addWidget(window.readiness_label, 2)
    status_row.addWidget(window.connection_label, 1)
    header_layout.addLayout(status_row)
    _put(header_layout, window.status_label)
    if groups.get('hardware') is not None:
        # Physical arm/disarm retains its original signals and explicit scope.
        _put(header_layout, groups['hardware'])
    outer.addWidget(header)

    body = QHBoxLayout()
    body.setSpacing(0)
    navigation_frame = QFrame()
    navigation_frame.setObjectName('workspaceNavigation')
    navigation_frame.setMinimumWidth(185)
    navigation_frame.setMaximumWidth(245)
    navigation_layout = QVBoxLayout(navigation_frame)
    navigation_layout.setContentsMargins(10, 14, 10, 12)
    workspace_label = QLabel('ESPACIO DE TRABAJO')
    workspace_label.setObjectName('workspaceSubtitle')
    navigation_layout.addWidget(workspace_label)
    window.workspace_navigation = QListWidget()
    window.workspace_navigation.setObjectName('workspaceNavigationList')
    window.workspace_navigation.setAccessibleName('Funciones de la interfaz')
    window.workspace_navigation.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    navigation_layout.addWidget(window.workspace_navigation, 1)
    tip = QLabel('F11 · Pantalla completa\nEsc · Salir de pantalla completa')
    tip.setWordWrap(True)
    tip.setObjectName('workspaceSubtitle')
    navigation_layout.addWidget(tip)
    body.addWidget(navigation_frame)
    workspace = QWidget()
    workspace_layout = QVBoxLayout(workspace)
    workspace_layout.setContentsMargins(18, 12, 12, 10)
    title_row = QHBoxLayout()
    window.workspace_page_title = QLabel()
    window.workspace_page_title.setObjectName('workspacePageTitle')
    title_row.addWidget(window.workspace_page_title, 1)
    window.workspace_detach_button = QPushButton('Abrir en ventana')
    title_row.addWidget(window.workspace_detach_button)
    workspace_layout.addLayout(title_row)
    window.workspace_pages = QStackedWidget()
    workspace_layout.addWidget(window.workspace_pages, 1)
    body.addWidget(workspace, 1)
    outer.addLayout(body, 1)
    window.workspace_page_ids = []
    window.workspace_page_layouts = {}
    window.workspace_detached_dialogs = {}

    joints = _page(window, 'joints', 'Articular / Manual')
    state_group = groups.get('state')
    if state_group is not None:
        state_group.layout().setColumnStretch(4, 1)
        state_group.layout().setColumnMinimumWidth(4, 120)
    window.manual_subviews = QTabWidget()
    objective = QWidget()
    objective_layout = QVBoxLayout(objective)
    objective_layout.setContentsMargins(4, 4, 4, 4)
    objective_controls = QWidget()
    controls_layout = QVBoxLayout(objective_controls)
    controls_layout.setContentsMargins(0, 0, 0, 0)
    _put(controls_layout, state_group, groups.get('command'), window.send_button)
    controls_layout.addStretch()
    window.velocity_table.setHorizontalHeaderLabels([
        'Articulación', 'Desplazamiento', 'Velocidad\nsolicitada',
        'Límite\npermitido', 'Previsión',
    ])
    window.velocity_table.setMinimumHeight(
        window.velocity_table.horizontalHeader().sizeHint().height()
        + sum(window.velocity_table.rowHeight(row)
              for row in range(window.velocity_table.rowCount())) + 32
    )
    preview_column = QWidget()
    preview_layout = QVBoxLayout(preview_column)
    preview_layout.setContentsMargins(0, 0, 0, 0)
    window.velocity_summary_label.setWordWrap(True)
    _put(preview_layout, groups.get('preview'))
    preview_layout.addStretch()
    window.objective_columns = _ResponsiveColumns(objective_controls, preview_column)
    objective_layout.addWidget(window.objective_columns)
    objective_layout.addStretch()
    window.manual_subviews.addTab(objective, 'Objetivo articular')
    joint_panel = window.manual_joint_panel or groups.get('manual') or groups.get('jog')
    window.manual_subviews.addTab(joint_panel, 'Manual · Joints')
    if window.manual_cartesian_panel is not None:
        window.manual_subviews.addTab(window.manual_cartesian_panel, 'Manual · XYZ')

    def stop_on_subview_change(_index):
        for button in (window.jog_mode_button, window.cartesian_jog_mode_button):
            if button is not None and button.isChecked():
                button.setChecked(False)
        scroll = window.workspace_pages.widget(
            window.workspace_page_ids.index('joints')
        )
        scroll.verticalScrollBar().setValue(0)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.verticalScrollBar().setEnabled(True)

    window.manual_subviews.currentChanged.connect(stop_on_subview_change)
    window.manual_metric_summary = QLabel('Esperando métricas de seguridad.')
    window.manual_metric_summary.setWordWrap(True)
    window.manual_metric_summary.setObjectName('safetyBanner')
    joints.addWidget(window.manual_metric_summary)
    joints.addWidget(window.manual_subviews)
    window.manual_diagnostic_summary = QLabel('Esperando diagnóstico.')
    window.manual_diagnostic_summary.setWordWrap(True)
    joints.addWidget(window.manual_diagnostic_summary)
    connection_view = QWidget()
    connection_layout = QGridLayout(connection_view)
    connection_layout.setContentsMargins(0, 0, 0, 0)
    connection_layout.setVerticalSpacing(1)
    window.manual_connection_indicators = {}
    for index, name in enumerate(window.connection_indicators):
        label = QLabel()
        window.manual_connection_indicators[name] = label
        connection_layout.addWidget(label, index // 4, index % 4)
    joints.addWidget(connection_view)

    window.workspace_details_toggle = QPushButton('Mostrar detalles de resumen y diagnóstico')
    window.workspace_details_toggle.setCheckable(True)
    joints.addWidget(window.workspace_details_toggle)
    window.workspace_details_widget = QWidget()
    details_layout = QVBoxLayout(window.workspace_details_widget)
    details_layout.setContentsMargins(0, 0, 0, 0)
    _put(details_layout, groups.get('metrics'), groups.get('pose'), groups.get('prediction'))
    window.workspace_open_diagnostic_button = QPushButton('Abrir diagnóstico en ventana')
    details_layout.addWidget(window.workspace_open_diagnostic_button)
    diagnostic_container = QWidget()
    window.workspace_diagnostic_container = QVBoxLayout(diagnostic_container)
    window.workspace_diagnostic_container.setContentsMargins(0, 0, 0, 0)
    window.workspace_diagnostic_content = QWidget()
    diagnostic_layout = QVBoxLayout(window.workspace_diagnostic_content)
    diagnostic_layout.setContentsMargins(0, 0, 0, 0)
    _put(diagnostic_layout, groups.get('connections'), window.preview_button,
         window.preview_note, window.volume_mode_label)
    window.workspace_diagnostic_container.addWidget(window.workspace_diagnostic_content)
    details_layout.addWidget(diagnostic_container)
    joints.addWidget(window.workspace_details_widget)
    window.workspace_details_widget.hide()
    window.workspace_details_toggle.toggled.connect(window.workspace_details_widget.setVisible)
    window.workspace_open_diagnostic_button.clicked.connect(
        lambda: detach_workspace_page(window, 'diagnostics')
    )
    joints.addStretch()

    if groups.get('cartesian') is not None:
        cartesian = _page(window, 'cartesian', 'Objetivo cartesiano')
        _put(cartesian, groups['cartesian'])
        cartesian.addStretch()
    if groups.get('trajectory') is not None:
        trajectory = _page(window, 'trajectory', 'Trayectorias')
        _put(trajectory, groups['trajectory'])
        trajectory.addStretch()

    history = _page(window, 'history', 'Historial')
    _put(history, groups.get('history'))
    history.addStretch()

    def navigate(index):
        if index < 0:
            return
        # Leaving manual inputs clears their intent through existing callbacks.
        # It never starts, cancels or redirects a point/trajectory command.
        if window.workspace_page_ids[index] != 'joints':
            for button in (window.jog_mode_button, window.cartesian_jog_mode_button):
                if button is not None and button.isChecked():
                    button.setChecked(False)
        window.workspace_pages.setCurrentIndex(index)
        window.workspace_page_title.setText(
            window.workspace_navigation.item(index).text()
        )
        window.workspace_detach_button.setVisible(
            window.workspace_page_ids[index] == 'history'
        )

    window.workspace_navigation.currentRowChanged.connect(navigate)
    window.workspace_detach_button.clicked.connect(
        lambda: detach_workspace_page(
            window, window.workspace_page_ids[window.workspace_navigation.currentRow()]
        )
    )
    window.setCentralWidget(shell)
    old_central.hide()
    old_central.deleteLater()
    window.setStyleSheet(WARM_STYLESHEET)
    window.workspace_navigation.setCurrentRow(0)
    window.workspace_fullscreen_shortcut = QShortcut(QKeySequence('F11'), window)
    window.workspace_fullscreen_shortcut.activated.connect(
        lambda: window.showMaximized() if window.isFullScreen() else window.showFullScreen()
    )
    window.workspace_escape_shortcut = QShortcut(QKeySequence('Esc'), window)
    window.workspace_escape_shortcut.activated.connect(
        lambda: window.showMaximized() if window.isFullScreen() else None
    )
