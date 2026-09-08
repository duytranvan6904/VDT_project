import subprocess


def agent_is_running(serial_port):
    pattern = f'MicroXRCEAgent serial --dev {serial_port}'
    try:
        output = subprocess.check_output(['pgrep', '-f', pattern])
        return len(output.strip()) > 0
    except (subprocess.CalledProcessError, OSError):
        return False


def build_agent_command(serial_port, baudrate):
    return ['MicroXRCEAgent', 'serial', '--dev', serial_port, '-b', str(baudrate)]


def spawn_agent(serial_port, baudrate):
    cmd = build_agent_command(serial_port, baudrate)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)