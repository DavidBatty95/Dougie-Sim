"""Optional container graphics selection and child-process OpenGL diagnostics.

The host-side selector uses only the standard library. OpenGL probing happens in
a separate process, never in the simulation/control process.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

MODES = ('auto', 'nvidia', 'dri', 'software', 'system')


def nvidia_available():
    if shutil.which('nvidia-smi') is None:
        return False
    try:
        gpu = subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
                             capture_output=True, text=True, timeout=5, check=True)
        runtimes = subprocess.run(['docker', 'info', '--format', '{{json .Runtimes}}'],
                                  capture_output=True, text=True, timeout=5, check=True)
        return bool(gpu.stdout.strip()) and 'nvidia' in json.loads(runtimes.stdout)
    except (OSError, ValueError, subprocess.SubprocessError):
        return False


def select_graphics(mode, needed):
    if mode not in MODES:
        raise ValueError(f'graphics must be one of {", ".join(MODES)}')
    if mode != 'auto':
        return mode
    if not needed:
        return 'system'
    if nvidia_available():
        return 'nvidia'
    devices = Path('/dev/dri')
    if devices.is_dir() and (any(devices.glob('renderD*')) or any(devices.glob('card*'))):
        return 'dri'
    return 'system'


def compose_graphics(root, mode):
    """Return Compose file arguments and `run` flags for the selected renderer."""
    if mode == 'nvidia':
        return ['-f', str(Path(root)/'docker-compose.gpu.yml')], []
    if mode == 'dri':
        devices = Path('/dev/dri')
        if not devices.is_dir():
            raise ValueError('--graphics dri requires /dev/dri on the Docker host')
        # Numeric host groups work for non-root container users without changing
        # host permissions or requiring matching group names in the image.
        render = sorted(devices.glob('renderD*'))
        cards = sorted(devices.glob('card*'))
        if not render and not cards:
            raise ValueError('no DRM render/card devices found in /dev/dri')
        os.environ['DOUGIE_DRI_RENDER_GID'] = str((render or cards)[0].stat().st_gid)
        os.environ['DOUGIE_DRI_CARD_GID'] = str((cards or render)[0].stat().st_gid)
        return ['-f', str(Path(root)/'docker-compose.dri.yml')], []
    if mode == 'software':
        return [], ['-e', 'LIBGL_ALWAYS_SOFTWARE=1', '-e', '__GLX_VENDOR_LIBRARY_NAME=mesa']
    return [], []


def probe():
    import glfw
    from OpenGL.GL import GL_RENDERER, GL_VENDOR, GL_VERSION, glGetString
    if not glfw.init():
        raise RuntimeError('OpenGL initialization failed; check DISPLAY and XAUTHORITY')
    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(64, 64, 'Dougie OpenGL diagnostic', None, None)
        if window is None:
            raise RuntimeError('could not create an OpenGL context')
        glfw.make_context_current(window)
        result = {name: glGetString(enum).decode('utf-8', errors='replace') for name, enum in
                  (('vendor', GL_VENDOR), ('renderer', GL_RENDERER), ('version', GL_VERSION))}
        result['software_renderer'] = any(name in result['renderer'].lower()
            for name in ('llvmpipe', 'softpipe', 'swrast', 'software rasterizer'))
        return result
    finally:
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


def rviz_command(config, report):
    return [sys.executable, '-m', 'dougie_sim.graphics', '--report', str(report), '--rviz',
            '-d', str(config), '--ros-args', '-p', 'use_sim_time:=true']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--require-hardware', action='store_true')
    parser.add_argument('--rviz', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        result = probe()
    except Exception as exc:
        parser.exit(1, f'Graphics probe failed: {exc}\n')
    print(json.dumps(result, indent=2), flush=True)
    if args.report:
        args.report.write_text(json.dumps(result, indent=2)+'\n')
    if args.require_hardware and result['software_renderer']:
        parser.exit(1, 'Software rendering detected. Try --graphics nvidia or --graphics dri.\n')
    if args.rviz is not None:
        if result['software_renderer']:
            print('RViz is using CPU software rendering; run ./dougie graphics-check to inspect acceleration.', flush=True)
        os.execvp('rviz2', ['rviz2', *args.rviz])


if __name__ == '__main__':
    main()
