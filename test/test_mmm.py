# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
"""ROS-free MMM transactions and pose/save decisions."""

import pytest
from test_operator_workflow import workflow


def protocol():
    events = []
    core = workflow.MmmProtocol(
        lambda n: events.append(('send', n)),
        lambda n: events.append(('dispatch', n)),
        lambda text: events.append(('warn', text)))
    return core, events


@pytest.mark.parametrize('number', [2, 4])
def test_even_three_legs_and_duplicate_late_frames(number):
    core, events = protocol()
    assert core.request(number, 0)
    core.receive(number, .1)
    core.receive(number, .2)
    core.receive(number, 4)
    assert events == [('send', number), ('send', number)]
    assert core.pending is None
    assert not core.request(number, 4)
    assert core.request(number, 5.1)


def test_even_retry_timeout_and_other_id():
    core, events = protocol()
    core.last_ping = 100
    core.request(2, 0)
    core.receive(4, .1)
    for i in range(1, 11):
        core.tick(i / 2)
    assert events[:10] == [('send', 2)] * 10
    assert events[10][0] == 'warn'
    assert core.pending is None and core.failed == 2
    core.receive(2, 6)
    assert len(events) == 11
    assert not core.request(2, 9)
    assert core.request(2, 10)


@pytest.mark.parametrize('number', [1, 3])
def test_odd_echo_then_dispatch_without_final_leg_echo(number):
    core, events = protocol()
    core.receive(number, 0)
    core.receive(number, .1)
    core.receive(number, 1)
    core.receive(number, 4)
    assert events == [('send', number), ('dispatch', number)]
    core.receive(number, 9)
    assert events == [('send', number), ('dispatch', number)] * 2


def test_ping_zero_has_no_third_leg_and_no_echo_loops():
    core, events = protocol()
    core.tick(0)
    core.receive(0, .1)
    core.receive(200, .2)
    core.tick(.5)
    assert events == [('send', 0)]
    core.tick(1)
    assert events == [('send', 0), ('send', 0)]


@pytest.mark.parametrize('at_start,busy', [(True, False), (False, False), (True, True)])
def test_save_ack_precedes_asynchronous_dispatch(at_start, busy):
    events = []
    core = workflow.MmmProtocol(
        lambda n: events.append(('ack', n)),
        lambda n: workflow.mmm_save(
            at_start, lambda snapshot: events.append(('launch', snapshot)), busy,
            lambda text: events.append(('warn', text))), lambda text: None)
    core.receive(3, 0)
    assert events[0] == ('ack', 3)
    if busy:
        assert len(events) == 2
    else:
        assert ('launch', not at_start) in events
    core.receive(3, .1)
    assert sum(e[0] == 'ack' for e in events) == 1
    assert sum(e[0] == 'launch' for e in events) == (0 if busy else 1)


def settle(gate, x=0, yaw=0, start=0):
    for i in range(22):
        now = start + i * .1
        gate.update([x, 0, yaw], now, now)
    return gate.at_start


def test_ready_mapping_hysteresis_and_parked_return():
    gate = workflow.StartGate({'root': [0, 0, 0]})
    assert not settle(gate, .14)
    assert settle(gate, .11, start=3)
    assert gate.update([.14, 0, 0], 5.2, 5.2)
    assert not gate.update([.151, 0, 0], 5.3, 5.3)
    assert not settle(gate, .14, start=6)
    assert settle(gate, .1, start=9)
    assert not gate.update([0, 0, .151], 11.2, 11.2)


def test_stale_repeated_nonfinite_and_backwards_tf_cannot_make_ready():
    gate = workflow.StartGate({'root': [0, 0, 0]})
    for now in (0, .1, .4, 3):
        assert not gate.update([0, 0, 0], 0, now)
    assert settle(gate, start=4)
    assert not gate.update([float('nan'), 0, 0], 6.2, 6.2)
    assert settle(gate, start=7)
    assert not gate.update([0, 0, 0], 1, 1)


def test_validated_save_retains_full_point_fifteen_tolerance():
    gate = workflow.StartGate({'root': [0, 0, 0]}, inward=.15)
    assert settle(gate, .15, .15)


@pytest.mark.parametrize('command', ['save', 'snapshot'])
def test_save_cli_keeps_incomplete_output_separate(monkeypatch, tmp_path, command):
    import json
    import sys

    captured = []
    (tmp_path / 'session.json').write_text(json.dumps({'frame_mode': 'room'}))
    monkeypatch.setattr(workflow, 'save', captured.append)
    monkeypatch.setattr(sys, 'argv', ['operator_workflow', command, str(tmp_path)])
    workflow.main()
    args = captured[0]
    assert args.unvalidated == (command == 'snapshot')
    assert args.room
    assert args.boot == tmp_path / 'boot.json'
    if command == 'save':
        assert args.output == tmp_path / 'snapshot'
    else:
        assert args.output.name.startswith('incomplete-')


def test_coordinator_waits_for_session_and_survives_bridge_restart(monkeypatch, tmp_path):
    import json
    import sys
    from types import SimpleNamespace as NS

    events, processes = [], []
    state = {'step': 0, 'callback': None, 'session': None}
    clock = NS(nanoseconds=10_000_000_000, to_msg=lambda: None)
    tf = NS(header=NS(stamp=NS(sec=10, nanosec=0)), transform=NS(
        translation=NS(x=0, y=0), rotation=NS(x=0, y=0, z=0, w=1)))

    class Child:
        pid = 999999

        def __init__(self, command, **kwargs):
            self.name = command[2] if command[0] == sys.executable else 'stop'
            events.append(('launch', self.name))
            self.code = None if self.name == 'mapping' else 0
            if self.name == 'mapping':
                state['session'] = workflow.Path(command[3])
                state['session'].mkdir(parents=True)
            if self.name == 'boot-pose':
                assert (state['session'] / 'session.json').exists()
                (state['session'] / 'boot.json').write_text(json.dumps({'root': [0, 0, 0]}))
            processes.append(self)

        def poll(self):
            return self.code

        def wait(self, **kwargs):
            self.code = 0

    def subscription(kind, topic, callback, qos):
        state['callback'] = callback

    def spin(node, **kwargs):
        state['step'] += 1
        step = state['step']
        if step == 1:
            state['callback'](NS(number=1))
        elif step == 3:
            (state['session'] / 'session.json').write_text('{}')
        elif step == 5:
            state['callback'](NS(number=2))
        elif step == 6:
            state['callback'](NS(number=3))

    node = NS(
        get_clock=lambda: NS(now=lambda: clock),
        get_logger=lambda: NS(error=lambda text: events.append(('warn', text))),
        create_publisher=lambda *a: NS(publish=lambda m: events.append(('send', m.number))),
        create_subscription=subscription)
    runtime = NS(node=node, subscriptions=[], close=lambda: events.append(('close', None)),
                 ros=NS(ok=lambda: state['step'] < 8, spin_once=spin))
    monkeypatch.setattr(workflow, 'Runtime', lambda sim: runtime)
    monkeypatch.setattr(workflow.subprocess, 'Popen', Child)
    monkeypatch.setattr(workflow.os, 'killpg', lambda *a: None)
    monkeypatch.setitem(sys.modules, 'dji_serial_bridge.msg', NS(Ping=lambda: NS(header=NS())))
    monkeypatch.setitem(sys.modules, 'rclpy.time', NS(Time=NS))
    monkeypatch.setitem(sys.modules, 'tf2_ros', NS(
        Buffer=lambda: NS(lookup_transform=lambda *a: tf),
        TransformListener=lambda *a: None, TransformException=RuntimeError))
    workflow.mmm(NS(sim_time=False, timeout=60, max_age=30, sessions=tmp_path,
                    stop_command='stop-service', room=True, no_referee=False, launch_args=[]))
    assert events.index(('send', 1)) < events.index(('launch', 'stop'))
    assert events.index(('launch', 'mapping')) < events.index(('launch', 'boot-pose'))
    assert events.index(('send', 3)) < events.index(('launch', 'save'))
    assert [e for e in events if e == ('send', 2)] == [('send', 2)] * 2
    assert all(p.poll() is not None for p in processes)
