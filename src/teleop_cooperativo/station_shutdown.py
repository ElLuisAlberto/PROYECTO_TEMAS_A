"""Signal only the launcher that created this station's desktop process."""
from __future__ import annotations

import os
from pathlib import Path
import signal


def signal_station_launcher() -> None:
    try:
        pid = int(os.environ['TEMAS_A_BASE_PID'])
        expected_start = int(os.environ['TEMAS_A_BASE_START_TICKS'])
        if pid <= 1 or pid == os.getpid():
            raise ValueError('PID inválido')
        proc = Path('/proc') / str(pid)
        if proc.stat().st_uid != os.getuid():
            raise ValueError('el propietario no coincide')
        stat = (proc / 'stat').read_text()
        # The process name is parenthesized and can itself contain spaces.
        fields = stat[stat.rfind(')') + 2:].split()
        if int(fields[19]) != expected_start:
            raise ValueError('el proceso ya no es la estación original')
        arguments = (proc / 'cmdline').read_bytes().split(b'\0')
        if not any(arg.endswith(b'scripts/start_base.sh') for arg in arguments):
            raise ValueError('el proceso no es el lanzador de la estación')
        os.kill(pid, signal.SIGTERM)
    except (KeyError, ValueError, OSError, IndexError) as error:
        raise RuntimeError(f'No se pudo cerrar el lanzador de la estación: {error}') from error
