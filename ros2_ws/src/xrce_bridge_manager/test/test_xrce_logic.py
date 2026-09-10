from unittest.mock import Mock, patch

from xrce_bridge_manager.xrce_logic import (
    agent_is_running,
    build_agent_command,
    spawn_agent,
)


def test_build_agent_command():
    assert build_agent_command('/dev/ttyAMA0', 921600) == [
        'MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'
    ]


@patch('xrce_bridge_manager.xrce_logic.subprocess.check_output')
def test_agent_is_running_when_pgrep_finds_process(check_output):
    check_output.return_value = b'1234 MicroXRCEAgent serial --dev /dev/ttyAMA0'
    assert agent_is_running('/dev/ttyAMA0')
    check_output.assert_called_once_with([
        'pgrep', '-f', 'MicroXRCEAgent serial --dev /dev/ttyAMA0'
    ])


@patch('xrce_bridge_manager.xrce_logic.subprocess.check_output')
def test_agent_is_not_running_on_pgrep_failure(check_output):
    check_output.side_effect = OSError('pgrep unavailable')
    assert not agent_is_running('/dev/ttyAMA0')


@patch('xrce_bridge_manager.xrce_logic.subprocess.Popen')
def test_spawn_agent_starts_silent_process(popen):
    process = Mock()
    popen.return_value = process
    assert spawn_agent('/dev/ttyAMA0', 921600) is process
    popen.assert_called_once()
    command = popen.call_args.args[0]
    assert command == build_agent_command('/dev/ttyAMA0', 921600)
