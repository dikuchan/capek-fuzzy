import fuzzylite as fl
import capek as ck

import math
import logging

SHIFT_LATENCY = 7
WHEEL_RADIUS = 0.662
ABS_MINIMAL_SPEED = 3.000
ABS_SLIP = 0.900

logging.basicConfig(level=logging.INFO)


class GearControl:
    def __init__(self):
        self.shift_ticks = 0
        self.ups = [None, 8000, 8000, 8000, 8000, 8000, math.inf]
        self.downs = [None, -math.inf, 2500, 3000, 3000, 3500, 3500]

    def tick(self, gear: float, rpm: float) -> (int, int):
        gear, rpm = int(gear), int(rpm)

        self.shift_ticks += 1

        if not gear:
            return 1, 1
        elif self.shift_ticks < SHIFT_LATENCY:
            return gear, 0
        elif rpm < self.downs[gear]:
            self.shift_ticks = 0
            return gear - 1, 1
        elif rpm > self.ups[gear]:
            self.shift_ticks = 0
            return gear + 1, 1

        return gear, 0


class TargetSpeed:
    def __init__(self):
        self.engine = fl.FllImporter().from_file('./TargetSpeed.fll')

    def tick(self, track: [float]) -> float:
        self.engine.input_variable("Front").value = track[9]
        self.engine.input_variable("M5").value = max(track[8], track[10])
        self.engine.input_variable("M10").value = max(track[7], track[11])

        self.engine.process()

        speed = self.engine.output_variable("Speed").value

        return speed


class GasBreakControl:
    def __int__(self):
        pass

    @staticmethod
    def tick(speed: float, wheel_spin_velocity: [float], target: float) -> (float, float):
        speed = speed / 3.600
        gbv = 2.000 / (1.000 + math.exp(speed - target))

        if gbv >= 1:
            accel = gbv - 1
            brake = 0
        else:
            brake = 1 - gbv
            accel = 0

        # ABS filter.
        if speed < ABS_MINIMAL_SPEED:
            return accel, brake

        slip = WHEEL_RADIUS * sum(wheel_spin_velocity) / (4.000 * speed)
        if slip < ABS_SLIP:
            brake *= slip

        """
        speed_wheels = WHEEL_RADIUS * sum(wheel_spin_velocity) / (4.000 * speed)
        if speed - speed_wheels > 1.500:
            result -= (speed - speed_wheels - 1.500) / 5.000
        """

        return accel, brake


class SteeringControl:
    def __init__(self):
        self.engine = fl.FllImporter().from_file('./Steering.fll')

    def tick(self, track: [float]) -> float:
        front = track[9]
        m5 = max(track[8], track[10])
        m10 = max(track[7], track[11])

        front = front if front < 100.000 else 100.000
        m5 = m5 if m5 < 100.000 else 100.000
        m10 = m10 if m10 < 100.000 else 100.000

        self.engine.input_variable("Front").value = front
        self.engine.input_variable("M5").value = m5
        self.engine.input_variable("M10").value = m10

        self.engine.process()

        steer = self.engine.output_variable("Steer").value
        if track[11] > track[7]:
            steer = -steer

        return steer


class OpponentModifier:
    def __init__(self):
        self.tolerance_brake = [6.000, 6.500, 7.000, 7.500]
        self.tolerance_brake = self.tolerance_brake + [8.000] + self.tolerance_brake[::-1]
        self.tolerance_overtake = [10.000, 10.000, 10.000, 10.000, 10.000, 12.000, 14.000, 16.000, 18.000, 20.000]
        self.tolerance_overtake = self.tolerance_overtake + [20] + self.tolerance_overtake[::-1]
        self.increase_overtake = [-0.100, -0.100, -0.100, -0.100, -0.100, -0.120, -0.140, -0.160, -0.180, -0.200]
        self.increase_overtake = self.increase_overtake + [0.000] + [-x for x in self.increase_overtake[::-1]]

    def tick(self, opponents: [float], speed: float,
             accel: float, brake: float, steer: float) -> (float, float, float):
        # Check distance violation.
        distances = opponents[14:23]
        violated = False
        for tolerance, distance in zip(self.tolerance_brake, distances):
            if distance < tolerance:
                violated = True
                break
        violated &= speed > 70.000
        delta_brake = 0.000
        if violated:
            delta_brake = -0.500

        # Overtaking correction.
        delta_steer = 0.000
        distances = opponents[8:29]
        for delta, tolerance, distance in zip(self.increase_overtake, self.tolerance_overtake, distances):
            if distance < tolerance:
                delta_steer += delta

        steer += delta_steer
        if abs(delta_brake - 0.000) > 1e-6:
            accel = 0.000
            brake += delta_brake

        print(f'D (brake): {delta_brake}, D (steer): {delta_steer}')

        return accel, brake, steer


class BaseDriver(ck.Driver):
    gc = GearControl()
    ts = TargetSpeed()
    gb = GasBreakControl()
    sc = SteeringControl()
    om = OpponentModifier()

    def drive(self):
        if abs(self.state.track_position) > 1.000:
            self.control.meta = 1.000

        gear, clutch = self.gc.tick(self.state.gear, self.state.rpm)
        target_speed = self.ts.tick(self.state.track)
        accel, brake = self.gb.tick(self.state.speed_X,
                                    self.state.wheel_spin_velocity,
                                    target_speed)
        steer = self.sc.tick(self.state.track)
        accel, brake, steer = self.om.tick(self.state.opponents, self.state.speed_X,
                                           accel, brake, steer)

        self.control.accel = accel
        self.control.brake = brake
        self.control.steering = steer
        self.control.gear = gear
        self.control.clutch = clutch


client = ck.Client(verbosity=1)
client.run(driver=BaseDriver,
           angles=[-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90])
