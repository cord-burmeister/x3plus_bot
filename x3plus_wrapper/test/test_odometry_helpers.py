import math

from x3plus_wrapper.Mecanum_driver_X3Plus import yahboomcar_driver


def test_update_position_straight():
    x, y, theta = yahboomcar_driver.update_position(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 0.0) < 1e-9
    assert abs(theta - 0.0) < 1e-9


def test_compute_ticks_per_meter_formula():
    class Dummy:
        pass

    d = Dummy()
    d.wheel_radius = 0.04
    d.ticks_per_revolution = 2444.0

    value = yahboomcar_driver.compute_ticks_per_meter(d)
    expected = 1.0 / (2.0 * math.pi * d.wheel_radius) * d.ticks_per_revolution
    assert abs(value - expected) < 1e-9
