# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
"""Independent geometry, artifact and convergence failure checks without ROS."""

import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image
import pytest
import yaml

loader = importlib.machinery.SourceFileLoader(
    'operator_workflow', str(Path(__file__).parents[1] / 'scripts/operator_workflow')
)
spec = importlib.util.spec_from_loader(loader.name, loader)
workflow = importlib.util.module_from_spec(spec)
loader.exec_module(workflow)


def metadata():
    return {
        'image': 'map.pgm',
        'origin': [0.0, 0.0, 0.0],
        'resolution': 1.0,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }


@pytest.mark.parametrize(
    'boot,measured,heading,reported',
    [
        ((-4.067, 0.002, 0), (-4.3, 0.2), 0, 0),
        ((2, -1, 0.3), (4, 2), 1.2, -0.4),
        ((0, 0, 0), (0, 0), -math.pi, math.pi),
    ],
)
def test_registration_maps_actual_boot_root(boot, measured, heading, reported):
    correction, root = workflow.registration(boot, *measured, heading, reported)
    assert workflow.transform(boot[:2], correction) == pytest.approx(measured)
    assert workflow.angle(boot[2] + correction[2]) == pytest.approx(root[2])
    assert root[2] == pytest.approx(workflow.angle(heading - reported))


@pytest.mark.parametrize('yaw', [0, math.pi / 2, math.pi, -math.pi / 2])
def test_rotated_raster_preserves_cell_locations(yaw):
    pixels = np.array([[0, 254, 205], [254, 0, 254]], dtype=np.uint8)
    output, raster = workflow.rotate_grid(metadata(), pixels, (4, -3, yaw))
    assert output['origin'][2] == 0
    assert raster.size == pixels.size
    for row in range(2):
        for col in range(3):
            x, y = workflow.transform((col + 0.5, 2 - row - 0.5), (4, -3, yaw))
            dest_col = math.floor(x - output['origin'][0])
            dest_row = raster.shape[0] - 1 - math.floor(y - output['origin'][1])
            assert raster[dest_row, dest_col] == pixels[row, col]


def test_arbitrary_rotation_unknown_padding_and_source_yaw():
    meta = metadata()
    meta['origin'] = [2, -1, 0.2]
    output, raster = workflow.rotate_grid(meta, np.full((20, 30), 254), (3, 4, 0.4))
    assert output['origin'][2] == 0
    assert set(np.unique(raster)) == {205, 254}
    assert np.count_nonzero(raster == 254) == pytest.approx(600, abs=4)


def make_bundle(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    Image.fromarray(np.full((10, 10), 254, dtype=np.uint8)).save(source / 'map.pgm')
    (source / 'map.yaml').write_text(yaml.safe_dump(metadata()))
    for suffix in ('data', 'posegraph'):
        (source / f'map.{suffix}').write_bytes(b'graph-data')
    (source / 'boot.json').write_text(
        json.dumps(
            {
                'root': [2, 2, 0],
                'chassis_yaw': math.pi / 2,
                'team': 'red',
                'raw_odom': [0, 0],
                'session_id': 'test-session',
            }
        )
    )
    return source


def test_room_bundle_without_referee(tmp_path):
    source = make_bundle(tmp_path)
    boot = json.loads((source / 'boot.json').read_text())
    boot.update(frame_mode='room', team=None)
    (source / 'boot.json').write_text(json.dumps(boot))
    workflow.prepare_room(NS(source=source, output=tmp_path / 'room'))
    assert workflow.check(tmp_path / 'room')['frame_mode'] == 'room'
    boot['frame_mode'] = 'field'
    (source / 'boot.json').write_text(json.dumps(boot))
    with pytest.raises(ValueError, match='referee team'):
        workflow.validate_bundle(source)


@pytest.mark.parametrize(
    'room,no_referee,team,passes',
    [
        (True, False, None, True),
        (False, False, None, False),
        (False, True, 'red', True),
        (False, True, 'blue', True),
    ],
)
def test_robot_id_zero(monkeypatch, room, no_referee, team, passes):
    import sys

    for name in ('nav_msgs.msg', 'dji_serial_bridge.msg', 'std_msgs.msg'):
        monkeypatch.setitem(
            sys.modules, name, NS(Odometry=object, RefSysStatus=object, String=object)
        )
    monkeypatch.setitem(
        sys.modules,
        'rclpy.qos',
        NS(
            qos_profile_sensor_data=object(),
            QoSProfile=lambda **kw: None,
            DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
        ),
    )

    def subscribe(kind, topic, callback, qos):
        if topic == '/odom':
            callback(
                NS(
                    header=NS(stamp=NS(sec=1, nanosec=0)),
                    pose=NS(pose=NS(position=NS(x=0, y=0))),
                )
            )
        elif topic.endswith('ref_sys'):
            callback(NS(robot_id=0, is_on_blue_team=False))
        return object()

    def wait(predicate, timeout):
        if not predicate():
            raise RuntimeError('timeout')

    runtime = NS(
        subscriptions=[],
        wait=wait,
        node=NS(
            create_subscription=subscribe, get_clock=lambda: NS(now=lambda: NS(nanoseconds=10**9))
        ),
    )
    args = NS(room=room, no_referee=no_referee, team=team, timeout=1, max_age=3)
    if passes:
        result = workflow.observe_inputs(runtime, args, require_session=False)
        assert result['team'] == (None if room else team)
    else:
        with pytest.raises((RuntimeError, ValueError)):
            workflow.observe_inputs(runtime, args, require_session=False)


@pytest.mark.parametrize('team,sign', [('red', -1), ('blue', 1)])
def test_alignment_keeps_graph_boot_frame_and_seeds_root(tmp_path, team, sign):
    source = make_bundle(tmp_path)
    boot = json.loads((source / 'boot.json').read_text())
    boot.update(team=team, raw_odom=[0, 0])
    (source / 'boot.json').write_text(json.dumps(boot))
    output = tmp_path / 'field'
    args = NS(source=source, output=output, x=sign * 3.4, y=-0.3, yaw=math.pi / 2, team=team)
    workflow.align(args)
    manifest = workflow.check(output)
    assert manifest['initial_root'] == pytest.approx([sign * 3.4, -0.3, 0])
    assert (source / 'map.posegraph').read_bytes() == b'graph-data'
    assert not (output / 'field.posegraph').exists()
    assert yaml.safe_load((output / 'field.yaml').read_text())['origin'][2] == 0
    with pytest.raises(FileExistsError):
        workflow.align(args)


def test_alignment_trusts_boot_contract_for_blue_zero_telemetry(tmp_path):
    source = make_bundle(tmp_path)
    boot_path = source / 'boot.json'
    boot = json.loads(boot_path.read_text())
    boot.update(team='blue', raw_odom=[0, 0], chassis_yaw=0)
    boot_path.write_text(json.dumps(boot))
    original = boot_path.read_bytes()
    output = tmp_path / 'field'
    workflow.align(NS(source=source, output=output, x=None, y=None, yaw=math.pi / 2, team=None))
    assert boot_path.read_bytes() == original
    manifest = json.loads((output / 'registration.json').read_text())
    assert manifest['boot_chassis_yaw'] == pytest.approx(math.pi / 2)


def test_missing_graph_and_boot_are_not_success(tmp_path):
    source = make_bundle(tmp_path)
    (source / 'map.data').write_bytes(b'')
    with pytest.raises(ValueError, match='empty'):
        workflow.validate_bundle(source)
    (source / 'map.data').write_bytes(b'data')
    (source / 'boot.json').unlink()
    with pytest.raises(FileNotFoundError):
        workflow.validate_bundle(source)


def test_session_can_be_moved_as_one_directory(tmp_path):
    session = tmp_path / 'original'
    session.mkdir()
    source = make_bundle(session)
    workflow.align(
        NS(
            source=source,
            output=session / 'deployment',
            x=-3.53,
            y=-0.426,
            yaw=math.pi / 2,
            team=None,
        )
    )
    moved = tmp_path / 'moved'
    session.rename(moved)
    assert workflow.check(moved / 'deployment')['team'] == 'red'


def scan(count=360):
    return NS(
        ranges=[1.0] * count,
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2 * math.pi / (count - 1),
    )


def test_variable_lidar_count_and_angles_are_rejected():
    geometry = workflow.scan_geometry(scan())
    assert workflow.geometry_matches(geometry, workflow.scan_geometry(scan()))
    assert not workflow.geometry_matches(geometry, workflow.scan_geometry(scan(341)))
    changed = scan()
    changed.angle_increment *= 1.01
    with pytest.raises(ValueError, match='beam count'):
        workflow.scan_geometry(changed)


def pose(covariance=None, x=1.0, yaw=0.0):
    if covariance is None:
        covariance = np.diag([0.01, 0.01, 0, 0, 0, 0.01]).flatten()
    return NS(
        pose=NS(
            covariance=covariance,
            pose=NS(
                position=NS(x=x, y=2.0),
                orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        )
    )


def accepted(message):
    return workflow.sample_ok(message, [1, 2, 0], 0.25, 0.15, 0.4, 0.25)


def test_convergence_rejects_bad_covariance_and_wrong_hypothesis():
    assert accepted(pose())
    assert not accepted(pose(x=2))
    assert not accepted(pose(yaw=math.pi))
    for value in (float('nan'), float('inf'), -1, 1):
        cov = np.diag([value, 0.01, 0, 0, 0, 0.01]).flatten()
        assert not accepted(pose(cov))
    cov = np.diag([0.01, 0.01, 0, 0, 0, 0.01])
    cov[0, 1] = cov[1, 0] = 0.1
    assert not accepted(pose(cov.flatten()))


def test_room_has_no_field_inputs_and_keeps_boot_coordinates(tmp_path):
    source = make_bundle(tmp_path)
    target = tmp_path / 'room'
    workflow.prepare_room(NS(source=source, output=target))
    manifest = workflow.check(target)
    assert manifest['frame_mode'] == 'room'
    assert manifest['initial_root'] == [2, 2, 0]
    assert manifest['field_from_boot'] == [0, 0, 0]
    assert 'team' not in manifest
    _, original = workflow.read_map(source / 'map')
    _, prepared = workflow.read_map(target / 'field')
    assert np.array_equal(original, prepared)


@pytest.mark.parametrize(
    'command,expected',
    [
        (['mapping', 'SESSION', '--room', 'real_hardware:=false'], ['use_rf2o:=false']),
        (
            ['mapping', 'SESSION', '--raw-odom', 'real_hardware:=false'],
            ['use_rf2o:=false', 'seed_from_raw_odom:=true'],
        ),
        (
            ['mapping', 'SESSION', 'real_hardware:=false'],
            ['use_rf2o:=false', 'seed_from_raw_odom:=true', 'autosave_map:=true'],
        ),
        (
            ['mapping', 'SESSION', '--rf2o', 'real_hardware:=false'],
            ['use_rf2o:=true', 'seed_from_raw_odom:=true', 'autosave_map:=true'],
        ),
        (
            [
                'localize',
                'BUNDLE',
                'real_hardware:=false',
            ],
            [
                'use_rf2o:=false',
                'seed_from_raw_odom:=true',
                'amcl_initial_x:=-3.53',
                'amcl_initial_yaw:=0.0',
            ],
        ),
        (
            ['localize', 'BUNDLE', '--rf2o', 'real_hardware:=false'],
            ['use_rf2o:=true', 'amcl_initial_x:=-3.53'],
        ),
    ],
)
def test_launch_cli_trailing_arguments_and_independent_seeds(
    monkeypatch, tmp_path, command, expected
):
    import sys

    captured = []
    command = [str(tmp_path / 'session') if c == 'SESSION' else c for c in command]
    monkeypatch.setattr(sys, 'argv', ['operator_workflow', *command])
    monkeypatch.setattr(
        workflow,
        'check',
        lambda _: {'frame_mode': 'field', 'initial_root': [-4.3, 0.2, 0.2]},
    )
    monkeypatch.setattr(workflow, 'ensure_stack_stopped', lambda: None)
    monkeypatch.setattr(workflow, 'field_start', lambda args, extra: [-3.53, -0.426, 0.0])
    monkeypatch.setattr(workflow, 'run_stack', lambda args, session, cmd: captured.extend(cmd))
    workflow.main()
    assert 'real_hardware:=false' in captured
    for value in expected:
        assert value in captured


def test_room_localize_no_odometry_seed_and_rejects_rf2o_override(monkeypatch):
    import sys

    captured = []
    monkeypatch.setattr(
        workflow, 'check', lambda _: {'frame_mode': 'room', 'initial_root': [2, 2, 0]}
    )
    monkeypatch.setattr(workflow, 'ensure_stack_stopped', lambda: None)
    monkeypatch.setattr(workflow, 'run_stack', lambda args, session, cmd: captured.extend(cmd))
    monkeypatch.setattr(
        sys, 'argv', ['operator_workflow', 'localize', 'BUNDLE', 'real_hardware:=false']
    )
    workflow.main()
    assert 'use_rf2o:=false' in captured
    assert 'amcl_initial_x:=2' in captured
    captured.clear()
    monkeypatch.setattr(
        sys,
        'argv',
        ['operator_workflow', 'mapping', 'SESSION', '--room', 'use_rf2o:=true'],
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 1
    assert not captured
    for command in (['mapping', 'SESSION', '--room', '--rf2o'], ['localize', 'BUNDLE', '--rf2o']):
        monkeypatch.setattr(sys, 'argv', ['operator_workflow', *command])
        with pytest.raises(SystemExit) as result:
            workflow.main()
        assert result.value.code == 1
    monkeypatch.setattr(
        sys, 'argv', ['operator_workflow', 'mapping', 'SESSION', '--raw-odom', '--rf2o']
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 2
    assert not captured


@pytest.mark.parametrize(
    'team,x,yaw,reported',
    [
        ('green', -3.53, 0, 0),
        ('blue', -3.53, 0, 0),
        ('red', 3.53, 0, 0),
        ('red', 0, 0, 0),
        ('red', -3.53, math.pi, 0),
        ('red', -3.53, 180, 0),
        ('red', -3.53, 0, math.pi),
    ],
)
def test_field_rejects_team_sign_heading_and_degrees(team, x, yaw, reported):
    boot = {'team': 'red', 'raw_odom': [0, 0], 'chassis_yaw': reported}
    with pytest.raises(ValueError):
        workflow.validate_field(boot, team, x, -0.426, yaw)


@pytest.mark.parametrize('team,sign,heading', [('red', -1, math.pi / 2), ('blue', 1, math.pi / 2)])
def test_registration_known_wall_in_field_coordinates(team, sign, heading):
    boot = {'team': team, 'raw_odom': [0, 0], 'chassis_yaw': heading}
    workflow.validate_field(boot, team, sign * 3.4, -0.226, heading)
    correction, _ = workflow.registration(
        [sign * 3.53, -0.426, 0], sign * 3.4, -0.226, heading, heading
    )
    # A wall 1 m toward +x and 2 m toward +y from the parked root.
    assert workflow.transform((sign * 3.53 + 1, -0.426 + 2), correction) == pytest.approx(
        (sign * 3.4 + 1, -0.226 + 2)
    )


def test_stale_boot_or_not_returned_refused():
    boot = {'root': [-3.53, -0.426, 0], 'session_id': 'current'}
    workflow.verify_return(boot, [-3.53, -0.426, 0], 'current')
    with pytest.raises(ValueError, match='wrong-session'):
        workflow.verify_return(boot, [-3.53, -0.426, 0], 'old')
    with pytest.raises(ValueError, match='return to boot'):
        workflow.verify_return(boot, [-3, 0, 0], 'current')


def test_seed_mismatch_refused():
    boot = {'team': 'red', 'raw_odom': [0, 0]}
    workflow.verify_start(boot, dict(boot))
    with pytest.raises(ValueError, match='seed/team mismatch'):
        workflow.verify_start(boot, {'team': 'red', 'raw_odom': [1, 0]})
    with pytest.raises(ValueError, match='seed/team mismatch'):
        workflow.verify_start(boot, {'team': 'blue', 'raw_odom': [3.53, 0]})


@pytest.mark.parametrize('team,sign', [('red', -1), ('blue', 1)])
def test_boot_pi_over_two_allows_float32_serialization_noise(team, sign):
    yaw = float(np.float32(math.pi / 2))
    workflow.validate_field(
        {'team': team, 'raw_odom': [0, 0], 'chassis_yaw': yaw},
        team, sign * 3.53, -0.426, yaw
    )


@pytest.mark.parametrize('team,sign', [('red', -1), ('blue', 1)])
@pytest.mark.parametrize('yaw,reported', [(0, 0), (math.pi, math.pi), (-math.pi / 2, 0)])
def test_field_rejects_old_headings(team, sign, yaw, reported):
    boot = {'team': team, 'raw_odom': [0, 0], 'chassis_yaw': reported}
    with pytest.raises(ValueError):
        workflow.validate_field(boot, team, sign * 3.53, -0.426, yaw)


@pytest.mark.parametrize('team,sign', [('red', -1), ('blue', 1)])
def test_field_trusts_boot_contract_over_placeholder_chassis_yaw(team, sign):
    # POSE has no chassis yaw, so captures record 0; registration uses the pi/2 contract.
    boot = {'team': team, 'raw_odom': [0, 0], 'chassis_yaw': 0}
    workflow.validate_field(boot, team, sign * 3.53, -0.426, math.pi / 2)
    correction, root = workflow.registration(
        [sign * 3.53, -0.426, 0], sign * 3.53, -0.426, math.pi / 2, workflow.BOOT_CHASSIS_YAW
    )
    assert correction == pytest.approx((0, 0, 0))
    assert root == pytest.approx((sign * 3.53, -0.426, 0))


def test_registration_places_known_wall_pixels():
    meta = metadata()
    meta.update(resolution=0.1, origin=[-5, -1, 0])
    pixels = np.full((20, 30), 254, dtype=np.uint8)
    pixels[:, 20] = 0  # Wall x=-2.95, cell centres from y=-0.95 through 0.95.
    correction, _ = workflow.registration([-4.625, 0, 0], -4.5, 0.2, 0, 0)
    output, raster = workflow.rotate_grid(meta, pixels, correction)
    rows, cols = np.where(raster == 0)
    assert len(cols) == 20
    assert output['origin'][0] + (cols + 0.5) * 0.1 == pytest.approx(np.full(20, -2.825))
    ys = output['origin'][1] + (raster.shape[0] - rows - 0.5) * 0.1
    assert sorted(ys) == pytest.approx(np.arange(-0.75, 1.2, 0.1))


def test_negative_scientific_yaw_parses(monkeypatch, tmp_path):
    import sys

    captured = []
    monkeypatch.setattr(
        sys,
        'argv',
        [
            'operator_workflow',
            'align',
            str(tmp_path),
            '--x',
            '-3.53',
            '--y',
            '-0.426',
            '--yaw',
            '-2e-05',
        ],
    )
    monkeypatch.setattr(workflow, 'align', lambda args: captured.append(args.yaw))
    workflow.main()
    assert captured == [-2e-05]


@pytest.mark.parametrize('option', ['--odom-x', '--odom-y'])
def test_operator_seed_removed(monkeypatch, option):
    import sys

    monkeypatch.setattr(sys, 'argv', ['operator_workflow', 'mapping', 'SESSION', option, '99'])
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 2


def test_failed_localize_stops_owned_launch_without_pass(monkeypatch, tmp_path, capsys):
    import subprocess
    import sys

    processes = []
    real_popen = subprocess.Popen

    def launch(*args, **kwargs):
        process = real_popen(*args, **kwargs)
        processes.append(process)
        return process

    def fail(_args):
        raise RuntimeError('away from registered start')

    monkeypatch.setattr(workflow.subprocess, 'Popen', launch)
    monkeypatch.setattr(
        workflow, 'FOXGLOVE_COMMAND', [sys.executable, '-c', 'import time; time.sleep(60)']
    )
    monkeypatch.setattr(workflow, 'convergence', fail)
    with pytest.raises(RuntimeError, match='away from registered start'):
        workflow.run_stack(
            NS(command='localize'),
            tmp_path,
            [sys.executable, '-c', 'import time; time.sleep(60)'],
        )
    assert len(processes) == 2
    assert all(p.poll() is not None for p in processes)
    assert 'LOCALIZE PASS' not in capsys.readouterr().out


def test_foxglove_command_is_read_only():
    command = workflow.FOXGLOVE_COMMAND
    assert command[:4] == ['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge']
    parameters = [command[i + 1] for i, arg in enumerate(command) if arg == '-p']
    assert parameters == [
        'port:=8765',
        'address:=0.0.0.0',
        'capabilities:=[connectionGraph, assets]',
    ]


@pytest.mark.parametrize('signum', [workflow.signal.SIGINT, workflow.signal.SIGTERM])
def test_workflow_signal_stops_launch_and_bridge(monkeypatch, tmp_path, signum):
    import os
    import signal
    import subprocess
    import sys
    import time

    processes = []
    real_popen = subprocess.Popen
    command = [sys.executable, '-c', 'import time; time.sleep(60)']

    def launch(*args, **kwargs):
        process = real_popen(*args, **kwargs)
        processes.append(process)
        return process

    def interrupt(_args):
        time.sleep(0.1)
        os.kill(os.getpid(), signum)
        for process in processes:
            process.wait(timeout=5)

    previous = signal.getsignal(signum)
    monkeypatch.setattr(workflow.subprocess, 'Popen', launch)
    monkeypatch.setattr(workflow, 'FOXGLOVE_COMMAND', command)
    monkeypatch.setattr(workflow, 'convergence', interrupt)
    with pytest.raises(RuntimeError, match='stopped before'):
        workflow.run_stack(NS(command='localize'), tmp_path, command)
    assert len(processes) == 2
    assert all(p.poll() is not None for p in processes)
    assert signal.getsignal(signum) == previous


def test_bridge_failure_stops_launch_without_localize_pass(monkeypatch, tmp_path, capsys):
    import subprocess
    import sys
    import time

    processes = []
    real_popen = subprocess.Popen

    def launch(*args, **kwargs):
        process = real_popen(*args, **kwargs)
        processes.append(process)
        return process

    monkeypatch.setattr(workflow.subprocess, 'Popen', launch)
    monkeypatch.setattr(
        workflow, 'FOXGLOVE_COMMAND', [sys.executable, '-c', 'raise SystemExit(9)']
    )
    monkeypatch.setattr(workflow, 'convergence', lambda args: time.sleep(0.2))
    with pytest.raises(RuntimeError, match='Foxglove bridge exited 9'):
        workflow.run_stack(
            NS(command='localize'),
            tmp_path,
            [sys.executable, '-c', 'import time; time.sleep(60)'],
        )
    assert all(p.poll() is not None for p in processes)
    assert 'LOCALIZE PASS' not in capsys.readouterr().out


@pytest.mark.parametrize('signum', [workflow.signal.SIGINT, workflow.signal.SIGTERM])
def test_clean_stop_allows_bridge_to_exit_before_launch(monkeypatch, tmp_path, signum):
    import os
    import sys
    import threading

    program = (
        'import signal, sys, time\n'
        'def stop(*args):\n'
        '    time.sleep(float(sys.argv[1]))\n'
        '    sys.exit(0)\n'
        'signal.signal(signal.SIGINT, stop)\n'
        'signal.signal(signal.SIGTERM, stop)\n'
        'time.sleep(60)\n'
    )
    monkeypatch.setattr(workflow, 'FOXGLOVE_COMMAND', [sys.executable, '-c', program, '0'])
    monkeypatch.setattr(workflow, 'convergence', lambda args: None)
    timer = threading.Timer(0.3, lambda: os.kill(os.getpid(), signum))
    timer.start()
    try:
        workflow.run_stack(
            NS(command='localize'), tmp_path, [sys.executable, '-c', program, '0.3']
        )
    finally:
        timer.cancel()
        timer.join()


@pytest.mark.parametrize(
    'initial,paused,after,live,error',
    [
        (False, True, True, 'live', None),
        (True, True, True, 'live', None),
        (True, False, False, 'live', None),
        (False, True, False, 'live', 'did not change'),
        (False, True, True, 'other', 'session differs'),
    ],
)
def test_insertion_toggle_is_idempotent_and_verified(
    monkeypatch, tmp_path, initial, paused, after, live, error
):
    import sys

    (tmp_path / 'session.json').write_text(json.dumps({'session_id': 'live'}))
    state = {'value': initial, 'toggles': 0}

    def toggle(_request):
        state['toggles'] += 1
        state['value'] = after
        return NS(status=True)

    def get(_request):
        return NS(values=[NS(type=1, bool_value=state['value'])])

    def client(_kind, name):
        handler = toggle if name.endswith('pause_new_measurements') else get
        return NS(
            service_is_ready=lambda: True,
            call_async=lambda r: NS(done=lambda: True, result=lambda: handler(r)),
        )

    class Runtime:
        def __init__(self, _sim):
            self.subscriptions = []
            self.node = NS(
                create_client=client,
                create_subscription=(lambda kind, topic, callback, qos: callback(NS(data=live))),
            )

        def wait(self, predicate, timeout):
            assert predicate()

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, 'rcl_interfaces.srv', NS(GetParameters=NS(Request=NS)))
    monkeypatch.setitem(sys.modules, 'slam_toolbox.srv', NS(Pause=NS(Request=NS)))
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=object))
    monkeypatch.setitem(
        sys.modules,
        'rclpy.qos',
        NS(QoSProfile=lambda **kw: None, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
    )
    monkeypatch.setattr(workflow, 'Runtime', Runtime)
    args = NS(session=str(tmp_path), sim_time=False, timeout=1)
    if error:
        with pytest.raises((RuntimeError, ValueError), match=error):
            workflow.set_insertion(args, paused)
    else:
        workflow.set_insertion(args, paused)
        assert state['value'] == paused
    assert state['toggles'] == (0 if live != 'live' or initial == paused else 1)


def test_alignment_defaults_and_typo_rejection(tmp_path):
    source = make_bundle(tmp_path)
    workflow.align(
        NS(source=source, output=tmp_path / 'default', x=None, y=None, yaw=math.pi / 2, team=None)
    )
    assert workflow.check(tmp_path / 'default')['initial_root'] == [-3.53, -0.426, 0]
    with pytest.raises(ValueError, match='0.3 m'):
        workflow.align(
            NS(
                source=source,
                output=tmp_path / 'typo',
                x=-3.17,
                y=-0.426,
                yaw=math.pi / 2,
                team=None,
            )
        )


@pytest.mark.parametrize('room,next_step', [(True, 'prepare-room'), (False, 'align')])
def test_saved_session_next_step_matches_frame(monkeypatch, tmp_path, capsys, room, next_step):
    import sys

    (tmp_path / 'session.json').write_text(json.dumps({'frame_mode': 'room' if room else 'field'}))
    monkeypatch.setattr(workflow, 'save', lambda args: None)
    monkeypatch.setattr(sys, 'argv', ['operator_workflow', 'save', str(tmp_path)])
    workflow.main()
    output = capsys.readouterr().out
    assert 'operator_workflow ' + next_step in output
    assert 'MEASURED_X' not in output


def test_red_map_accepts_live_blue_start():
    assert workflow.verify_start(
        {'team': 'red', 'raw_odom': [0, 0]}, {'team': 'blue', 'raw_odom': [0, 0]}
    ) == [3.53, -0.426, 0]


def test_bootstrap_sigterm_cleans_process_group(monkeypatch):
    import os
    import signal
    import subprocess
    import sys

    processes = []
    real_popen = subprocess.Popen

    def launch(*a, **kw):
        process = real_popen([sys.executable, '-c', 'import time; time.sleep(60)'], **kw)
        processes.append(process)
        return process

    def observe(*a, **kw):
        os.kill(os.getpid(), signal.SIGTERM)

    previous = signal.getsignal(signal.SIGTERM)
    monkeypatch.setattr(workflow.subprocess, 'Popen', launch)
    monkeypatch.setattr(workflow, 'Runtime', lambda sim: NS(close=lambda: None))
    monkeypatch.setattr(workflow, 'observe_inputs', observe)
    with pytest.raises(RuntimeError, match='signal'):
        workflow.field_start(NS(sim_time=False), [])
    assert processes[0].poll() is not None
    assert signal.getsignal(signal.SIGTERM) == previous


def test_localize_gate_defaults_match_standalone(monkeypatch, tmp_path):
    import sys

    captured = []
    fields = ('max_std', 'max_yaw_std', 'max_distance', 'max_yaw', 'hold', 'samples', 'max_age')
    monkeypatch.setattr(
        workflow, 'convergence', lambda args: captured.append([getattr(args, k) for k in fields])
    )
    monkeypatch.setattr(sys, 'argv', ['operator_workflow', 'convergence', str(tmp_path)])
    workflow.main()
    monkeypatch.setattr(
        workflow.subprocess, 'Popen', lambda *a, **kw: NS(poll=lambda: 0, returncode=0)
    )
    with pytest.raises(RuntimeError, match='stopped before'):
        workflow.run_stack(NS(command='localize', max_age=30), tmp_path, [])
    assert captured[0] == captured[1]
    assert captured[1][-1] == 3.0


@pytest.mark.parametrize('code', [0, 1])
def test_unsaved_warning_on_launch_exit(monkeypatch, tmp_path, capsys, code):
    import sys

    monkeypatch.setitem(sys.modules, 'std_msgs.msg', NS(String=lambda **kw: kw))
    monkeypatch.setitem(
        sys.modules,
        'rclpy.qos',
        NS(QoSProfile=lambda **kw: None, DurabilityPolicy=NS(TRANSIENT_LOCAL=1)),
    )
    monkeypatch.setattr(
        workflow,
        'Runtime',
        lambda sim: NS(
            node=NS(create_publisher=lambda *a: NS(publish=lambda m: None)), close=lambda: None
        ),
    )
    monkeypatch.setattr(
        workflow.subprocess, 'Popen', lambda *a, **kw: NS(poll=lambda: code, returncode=code)
    )
    args = NS(command='mapping', sim_time=False, room=True, no_referee=False, team=None)
    if code:
        with pytest.raises(RuntimeError, match='launch exited'):
            workflow.run_stack(args, tmp_path, [])
    else:
        workflow.run_stack(args, tmp_path, [])
    output = capsys.readouterr().out
    assert output.count('WARNING: UNSAVED') == 1
    assert str(tmp_path / 'backup') in output
    metadata = json.loads((tmp_path / 'session.json').read_text())
    assert metadata['frame_mode'] == 'room' and metadata['team'] is None
