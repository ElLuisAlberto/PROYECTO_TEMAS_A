import unittest
from unittest.mock import Mock, patch

from PyQt5 import QtWidgets
from teleop_cooperativo.desktop_qt import QtDesktopConsole


class StationShutdownTest(unittest.TestCase):
    def setUp(self):
        self.console = QtDesktopConsole.__new__(QtDesktopConsole)
        self.console.window = None
        self.console.recovery_dialog = None
        self.console.shutdown_locked = False
        self.console.shutdown_detail = Mock()
        self.console.shutdown_button = Mock()
        self.control = Mock()
        self.control.shutdown_future = None
        self.control.shutdown_result = None
        self.control.shutdown_message = 'Deteniendo…'
        self.control.prepare_shutdown.return_value = True
        self.console.output_control = self.control

    def begin(self):
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.console.shutdown_station()
        self.assertTrue(self.console.shutdown_locked)

    def test_waits_for_bridge_before_signalling_launcher(self):
        self.begin()
        self.control.shutdown_future = Mock()
        with patch('teleop_cooperativo.station_shutdown.signal_station_launcher') as signal:
            self.console._finish_shutdown_if_ready()
            signal.assert_not_called()
            self.control.shutdown_future = None
            self.control.shutdown_result = True
            self.console._finish_shutdown_if_ready()
            signal.assert_called_once()

    def test_failed_stop_keeps_station_open_and_allows_retry(self):
        self.begin()
        self.control.shutdown_result = False
        self.control.shutdown_message = 'No se pudo confirmar la parada'
        with patch('teleop_cooperativo.station_shutdown.signal_station_launcher') as signal:
            self.console._finish_shutdown_if_ready()
            signal.assert_not_called()
        self.console.shutdown_button.setEnabled.assert_called_with(True)
        self.console.shutdown_detail.setText.assert_called_with('No se pudo confirmar la parada')

    def test_missing_service_does_not_signal_launcher(self):
        self.control.prepare_shutdown.return_value = False
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            with patch('teleop_cooperativo.station_shutdown.signal_station_launcher') as signal:
                self.console.shutdown_station()
                self.console._finish_shutdown_if_ready()
                signal.assert_not_called()
        self.assertFalse(self.console.shutdown_locked)


if __name__ == '__main__':
    unittest.main()
