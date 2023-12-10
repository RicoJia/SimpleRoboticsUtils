#!/usr/bin/env python3

from simple_robotics_python_utils.controllers.pid_controllers import (
    BasePIDController,
    IncrementalPIDController,
    FeedforwardIncrementalPIDController,
    PIDParams,
)
import numpy as np


class TestBasePIDController:
    def _get_params(self):
        return PIDParams(0.3, 0.2, 0.1), PIDParams(0.3, 0.2, 0.1)

    def test_controller_base_class(self):
        controller = BasePIDController(*self._get_params())
        controller.store_speed((0.1, 0.1))
        assert np.allclose(controller.get_actual_speeds(), np.array((0.1, 0.1)))
        try:
            controller.get_pwms()
        except NotImplementedError:
            pass

    def _test_pid_controller(
        self, controller: BasePIDController, positive_direction: bool
    ):
        def _single_direction_increase_and_overshoot_test(positive_direction: bool):
            sign = 1.0 if positive_direction else -1.0
            commanded_speed = sign * np.array((0.2, 0.2))
            controller.store_commanded_speed(tuple(commanded_speed))

            # Increasing output test
            # Creates increasing actual output
            # So exepcting output pwm to be the same sign as positive_direction
            increasing_actual_speed_multiplier_seq = [0.0, 0.5, 1.0]
            # because of the derivarive terms, pwm might dip a bit when it first gets there.
            for a in increasing_actual_speed_multiplier_seq:
                controller.store_speed(tuple(a * commanded_speed))
                pwm = np.asarray(controller.get_pwms()[0])
                # TODO Remember to remove
                print(
                    f"Rico: pwm, commanded, actual {pwm, commanded_speed, controller.get_actual_speeds()}"
                )
                assert np.all(np.sign(pwm) == np.sign(commanded_speed))

            # Overshoot test
            # Creates an overshoot but with, the same output at the beginning.
            # So expecting output pwm to go down to the other sign.
            overshoot_actual_speed_multiplier_seq = [1.0, 1.5, 2.0, 1.5]
            all_single_speeds = []
            for a in overshoot_actual_speed_multiplier_seq:
                controller.store_speed(tuple(a * commanded_speed))
                all_single_speeds.append(controller.get_pwms()[0])
            for i in range(len(all_single_speeds) - 1):
                if sign * (all_single_speeds[i + 1] - all_single_speeds[i]) > 0:
                    msg = f"the controller output should be either the other sign, or it decreases. output: {all_single_speeds}"
                    assert all_single_speeds[i + 1] * sign < 0, msg
            undershoot_actual_speed_multiplier_seq = [0.5, 0.3, 0.5]

        # Walk thru an example:
        # Flip on safety switches
        controller.start_getting_wheel_vel_updates()
        controller.start_taking_commands()
        _single_direction_increase_and_overshoot_test(positive_direction)

    def test_controllers(self):
        controller = IncrementalPIDController(*self._get_params())
        self._test_pid_controller(controller, positive_direction=True)
        controller = IncrementalPIDController(*self._get_params())
        self._test_pid_controller(controller, positive_direction=False)
