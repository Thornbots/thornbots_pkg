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
    core.receive(6 - number, 4.1)
    assert events == [('send', number), ('send', number)]
    assert core.pending is None
    assert core.completed_even == number
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
def test_odd_always_echoes_but_dispatches_only_after_quiet_window(number):
    core, events = protocol()
    core.receive(number, 0)
    core.receive(number, .1)
    core.receive(number, 1)
    core.receive(number, 4)
    assert events == [('send', number), ('dispatch', number)] + [('send', number)] * 3
    core.receive(number, 8.9)
    assert events[-1] == ('send', number)
    assert events.count(('dispatch', number)) == 1
    core.receive(number, 13.9)
    assert events[-2:] == [('send', number), ('dispatch', number)]
    assert events.count(('dispatch', number)) == 2


@pytest.mark.parametrize('number', [2, 4, 5, 200, 255])
def test_unsolicited_even_echoes_and_unknown_numbers_are_ignored(number):
    core, events = protocol()
    core.receive(number, 0)
    assert events == []
    assert core.pending is None


@pytest.mark.parametrize('number', [1, 2, 3, 4])
@pytest.mark.parametrize('drop', [None, 0, 1, 2, 3])
def test_simulated_mcb_single_frame_loss_terminates(number, drop):
    from collections import deque

    frames, work, transitions = deque(), [], []
    core = workflow.MmmProtocol(
        lambda n: frames.append(('C', n)), work.append,
        lambda text: pytest.fail(text))
    core.last_ping = 100  # Isolate the exchange from independent link checks.
    c_pending = number if number % 2 else None
    sent = 0
    dropped = False
    if c_pending:
        frames.append(('J', number))
    else:
        assert core.request(number, 0)
    for step in range(12):
        now = step * .5
        while frames:
            target, received = frames.popleft()
            index = sent
            sent += 1
            assert sent <= 6, 'echo loop failed to terminate'
            if index == drop:
                dropped = True
                continue
            if target == 'J':
                core.receive(received, now)
            elif received in (2, 4):
                frames.append(('J', received))
                if not transitions:
                    transitions.append(received)
            elif c_pending == received:
                c_pending = None
                frames.append(('J', received))
        if step < 10:
            core.tick(now)
            if c_pending:
                frames.append(('J', c_pending))
    assert not frames and c_pending is None and core.pending is None
    assert work == ([number] if number % 2 else [])
    assert transitions == ([] if number % 2 else [number])
    assert dropped == (drop is not None)


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
    assert sum(e[0] == 'ack' for e in events) == 2
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


@pytest.mark.parametrize('save_code,echo,at_start,expected', [
    (0, 2, True, 'save'), (1, 2, True, 'save'),
    (0, None, True, 'snapshot'), (0, 4, True, 'snapshot'),
    (0, 2, False, 'snapshot')])
def test_coordinator_waits_for_session_and_survives_bridge_restart(
        monkeypatch, tmp_path, capsys, save_code, echo, at_start, expected):
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
            self.name = command[2] if command[0] == sys.executable else command[0]
            events.append(('launch', self.name))
            self.code = None if self.name == 'mapping' else 0
            if self.name in ('save', 'snapshot'):
                self.code = None
            if self.name == 'shutdown':
                assert 'MMM SAVE COMPLETE:' in capsys.readouterr().out
                assert command == ['shutdown', 'two words']
                assert ('save-result', 0) in events
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
            if echo is not None:
                if echo == 4:
                    # Complete a later Mapping exchange, then return to start
                    # before its new Ready exchange has completed.
                    state['callback'](NS(number=2))
                    tf.transform.translation.x = 1
                else:
                    state['callback'](NS(number=echo))
        elif step == 6:
            if echo == 4:
                state['callback'](NS(number=4))

                def returned(self, *args):
                    self.at_start = True
                    return True
                monkeypatch.setattr(workflow.StartGate, 'update', returned)
                tf.transform.translation.x = 0
            if not at_start:
                tf.transform.translation.x = 1
            state['callback'](NS(number=3))
        elif step == 7:
            assert ('launch', 'shutdown') not in events
            for process in processes:
                if process.name in ('save', 'snapshot'):
                    events.append(('save-result', save_code))
                    process.code = save_code
            # C echoes the final leg too; no new work or completion update.
            state['callback'](NS(number=4 if echo == 2 else 2))

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
                    stop_command='stop-service', shutdown_command='shutdown "two words"',
                    room=True, no_referee=False, launch_args=[]))
    assert events.index(('send', 1)) < events.index(('launch', 'stop-service'))
    assert events.index(('launch', 'mapping')) < events.index(('launch', 'boot-pose'))
    assert events.index(('send', 3)) < events.index(('launch', expected))
    shutdowns = [e for e in events if e == ('launch', 'shutdown')]
    assert len(shutdowns) == int(expected == 'save' and save_code == 0)
    if shutdowns:
        assert events.index(('launch', 'save')) < events.index(shutdowns[0])
    if save_code:
        assert any('SAVE FAILED' in str(e) for e in events)
        assert state['step'] == 8
    assert all(p.poll() is not None for p in processes)
