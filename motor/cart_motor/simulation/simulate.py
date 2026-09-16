#!/usr/bin/env python3
"""Offline simulation of the actual SimpleMotor and Modbus driver (no hardware).

ROS messages and serial registers are mocked. Ideal acceleration-limited motion
uses configured speed/acceleration; stop commands stop instantly in this model.
No bus latency, load, gravity, current physics, or real drive firmware is modeled.
"""
import contextlib
import csv
import io
import json
import math
from pathlib import Path
import sys
import types

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


class Clock:
    now = 100.0
    motor = None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        steps = max(1, math.ceil(seconds / .001))
        dt = seconds / steps
        for _ in range(steps):
            if self.motor:
                self.motor.advance(dt)
            self.now += dt


clock = Clock()


class Message:
    def __init__(self, data=None):
        self.data = data


class Node:
    def __init__(self, name):
        pass

    def create_subscription(self, *args):
        pass

    def create_publisher(self, *args):
        return types.SimpleNamespace(publish=lambda msg: None)

    def get_logger(self):
        return types.SimpleNamespace(info=lambda msg: None,
                                     warn=lambda msg: None, error=lambda msg: None)


class Instrument:
    def __init__(self, port, slave):
        self.serial = types.SimpleNamespace(is_open=True,
            reset_input_buffer=lambda: None, reset_output_buffer=lambda: None)
        self.regs = {}
        self.pos = float(initial_position)
        self.velocity = 0.0
        self.target = None
        self.current = 1.0
        self.fault = 0
        self.stalled = False
        clock.motor = self

    def read_register(self, addr, *args, **kwargs):
        if addr == cfg.ADDR_P02_34:
            return self.fault
        if addr == cfg.ADDR_P0B_24:
            return round(self.current / cfg.CURRENT_SCALE)
        return self.regs.get(addr, 0)

    def read_registers(self, addr, count, **kwargs):
        if addr == cfg.ADDR_P0B_07:
            value = round(self.pos) & 0xffffffff
            return [value & 65535, value >> 16]
        return [self.regs.get(addr + i, 0) for i in range(count)]

    def write_registers(self, addr, words):
        for i, word in enumerate(words):
            self.regs[addr + i] = word

    def value(self, addr):
        n = self.regs.get(addr, 0) | (self.regs.get(addr + 1, 0) << 16)
        return n - 2**32 if n >= 2**31 else n

    def write_register(self, addr, value, **kwargs):
        self.regs[addr] = value
        if addr == cfg.ADDR_P0D_08:
            if value == 7:
                self.target = self.value(cfg.ADDR_P10_14)
            elif value in (256, 512):
                self.target = None
                self.velocity = 0.0

    def advance(self, dt):
        if self.target is None or self.stalled:
            return
        error = self.target - self.pos
        vmax = self.value(cfg.ADDR_P10_25)
        accel = self.value(cfg.ADDR_P10_27)
        decel = self.value(cfg.ADDR_P10_29)
        wanted = math.copysign(min(vmax, math.sqrt(2 * decel * abs(error))), error)
        old = self.velocity
        self.velocity += max(-accel * dt, min(accel * dt, wanted - old))
        step = (old + self.velocity) * .5 * dt
        if abs(step) >= abs(error):
            self.pos = float(self.target)
            self.velocity = 0.0
        else:
            self.pos += step


# Replace dependencies before importing production modules: no serial port opens.
sys.modules['serial'] = types.SimpleNamespace(PARITY_EVEN='E')
sys.modules['minimalmodbus'] = types.SimpleNamespace(Instrument=Instrument, MODE_RTU='rtu')
sys.modules['rclpy'] = types.ModuleType('rclpy')
sys.modules['rclpy.node'] = types.SimpleNamespace(Node=Node)
sys.modules['std_msgs'] = types.ModuleType('std_msgs')
sys.modules['std_msgs.msg'] = types.SimpleNamespace(
    Bool=Message, Int32=Message, String=Message, Float32=Message)
import motor_config as cfg
import motor_driver as drv
import simple_motor as app
drv.time = clock
app.time = clock


def run(name, events, duration, expected, initial=10000, start=10000, end=55000):
    global initial_position
    initial_position = initial
    cfg.START_POS, cfg.END_POS = start, end
    clock.now, clock.motor = 100.0, None
    drv.current_trip_count = 0
    node = app.SimpleMotor()
    rows, transitions = [], []
    error = None
    origin = clock.now
    try:
        node.setup()
        assert node.motor.target is None, "setup must not command movement"
        origin = clock.now
        pending = list(events)
        previous = None
        while clock.now - origin <= duration:
            elapsed = clock.now - origin
            while pending and pending[0][0] <= elapsed + 1e-9:
                _, action = pending.pop(0)
                if action in ('start', 'back', 'stop'):
                    getattr(node, 'on_' + action)(Message(True))
                elif action == 'overcurrent':
                    node.motor.current = cfg.MAX_CURRENT_A + 1
                elif action == 'fault':
                    node.motor.fault = 7
                elif action == 'stall':
                    node.motor.stalled = True
            keep_running = node.tick()
            elapsed = clock.now - origin
            rows.append((elapsed, (node.motor.pos - initial) / cfg.MOTOR_UNITS_PER_MM,
                         node.state))
            if node.state != previous:
                transitions.append([round(elapsed, 3), node.state])
                previous = node.state
            if not keep_running:
                break
            clock.sleep(cfg.POLL_S)
    except Exception as exc:
        error = str(exc)
        node.shutdown_motor(fault=True)
        transitions.append([round(clock.now - origin, 3), node.state])
    assert node.state == expected, (name, node.state, expected)
    if name in ('round_trip', 'reverse_coordinates'):
        assert [s for _, s in transitions] == ['READY', 'FORWARD_MOVING', 'AT_END', 'BACKWARD_MOVING', 'READY']
        assert abs(node.motor.pos - start) <= cfg.ARRIVAL_TOLERANCE
    if name == 'interrupt_return':
        assert 'AT_END' not in [s for _, s in transitions]
        assert abs(node.motor.pos - start) <= cfg.ARRIVAL_TOLERANCE
    if name == 'startup_return':
        assert transitions[0][1] == 'NOT_AT_START'
        assert 'FORWARD_MOVING' not in [s for _, s in transitions]
        assert abs(node.motor.pos - start) <= cfg.ARRIVAL_TOLERANCE
    if name in ('position_limit', 'unset_positions', 'overlapping_positions'):
        assert node.motor is None, 'invalid configuration must fail before connection'
    with (HERE / (name + '.csv')).open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'relative_position_mm', 'state'])
        writer.writerows(rows)
    return dict(scenario=name, transitions=transitions, final_state=node.state,
                final_position=round(node.motor.pos) if node.motor else None, error=error,
                start_position=start, end_position=end), rows


if __name__ == '__main__':
    scenarios = [
        ('round_trip', [(0.2, 'start'), (2.2, 'back')], 4.2, 'READY'),
        ('interrupt_return', [(0.2, 'start'), (.7, 'back')], 2.2, 'READY'),
        ('emergency_stop', [(0.2, 'start'), (.7, 'stop')], 2, 'STOPPED'),
        ('overcurrent', [(0.2, 'start'), (.4, 'overcurrent')], 3, 'FAULT'),
        ('driver_fault', [(0.2, 'start'), (.4, 'fault')], 2, 'FAULT'),
        ('timeout', [(0, 'stall'), (.2, 'start')], 32, 'FAULT'),
        ('position_limit', [], 1, 'FAULT', 10000, 10000, 320000),
        ('unset_positions', [], 1, 'FAULT', 10000, None, None),
        ('overlapping_positions', [], 1, 'FAULT', 10000, 10000, 10300),
        ('reverse_coordinates', [(0.2, 'start'), (2.2, 'back')], 4.2, 'READY', 10000, 10000, -35000),
        ('startup_return', [(0.2, 'start'), (.4, 'back')], 2, 'READY', 25000),
    ]
    results, traces = [], []
    with contextlib.redirect_stdout(io.StringIO()):
        for scenario in scenarios:
            result, trace = run(*scenario)
            results.append(result)
            traces.append(trace)
    (HERE / 'results.json').write_text(json.dumps(results, indent=2, ensure_ascii=False) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), constrained_layout=True)
    for ax, result, trace in zip(axes, results, traces):
        ax.plot([r[0] for r in trace], [r[1] for r in trace])
        ax.axhline((result['end_position'] - result['start_position']) / cfg.MOTOR_UNITS_PER_MM, color='gray', linestyle='--', linewidth=.8)
        ax.set(title=result['scenario'], ylabel='Travel from start (mm)', xlabel='Time after setup (s)')
        ax.grid(alpha=.25)
        for t, state in result['transitions']:
            ax.axvline(t, color='gray', alpha=.3)
            ax.text(t, 42, state, fontsize=7, rotation=15)
        ax.set_ylim(-2, 55)
    fig.suptitle('cart_motor offline simulation — ideal motion, no hardware')
    fig.savefig(HERE / 'motion.png', dpi=150)
    plt.close(fig)
    print(json.dumps(results, indent=2, ensure_ascii=False))
