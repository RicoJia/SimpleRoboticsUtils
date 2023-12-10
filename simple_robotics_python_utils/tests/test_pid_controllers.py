#!/usr/bin/env python3

"""
Notes:
1. This test is quite interesting, it provides a base performance test of a PID controller.

"""
from simple_robotics_python_utils.common.io import try_remove_file
from simple_robotics_python_utils.controllers.pid_controllers import (
    BasePIDController,
    IncrementalPIDController,
    FeedforwardIncrementalPIDController,
    PIDParams,
)
import numpy as np
import csv


class TestBasePIDController:
    def _get_params(self):
        return PIDParams(0.1, 0.4, 0.1), PIDParams(0.1, 0.4, 0.1)

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
            print(
                f"================== {type(controller)}, positive_direction: {positive_direction} ================= "
            )
            sign = 1.0 if positive_direction else -1.0
            commanded_speed = sign * np.array((0.3, 0.3))
            controller.store_commanded_speed(tuple(commanded_speed))

            # Increasing output test
            # Creates increasing actual output
            # So exepcting output pwm to be the same sign as positive_direction
            increasing_actual_speed_multiplier_seq = [0.0, 0.5, 1.0]
            # because of the derivarive terms, pwm might dip a bit when it first gets there.
            for a in increasing_actual_speed_multiplier_seq:
                controller.store_speed(tuple(a * commanded_speed))
                pwm = np.asarray(controller.get_pwms()[0])
                print(
                    f"Rico: pwm, commanded, actual {pwm, commanded_speed, controller.get_actual_speeds()}"
                )
                assert np.all(np.sign(pwm) == np.sign(commanded_speed))

            # Overshoot test
            # Creates an overshoot but with, the same output at the beginning.
            # So expecting output pwm to go down to the other sign.
            overshoot_actual_speed_multiplier_seq = [1.0, 1.5, 2.0, 1.5]
            all_single_pwms = []
            for a in overshoot_actual_speed_multiplier_seq:
                print(f"\n")
                controller.store_speed(tuple(a * commanded_speed))
                pwm = controller.get_pwms()[0]
                all_single_pwms.append(pwm)
                print(
                    f"Rico: pwm, commanded, actual {pwm, commanded_speed, controller.get_actual_speeds()}"
                )

            for i in range(len(all_single_pwms) - 1):
                if sign * (all_single_pwms[i + 1] - all_single_pwms[i]) >= 0:
                    msg = (
                        "The controller output should be either the other sign, or it decreases. "
                        + "This could be a tuning issue, as well"
                        + f"output: {all_single_pwms}"
                    )
                    assert all_single_pwms[i + 1] * sign < 0, msg

        # Walk thru an example:
        # Flip on safety switches
        controller.start_getting_wheel_vel_updates()
        controller.start_taking_commands()
        _single_direction_increase_and_overshoot_test(positive_direction)

    def test_incremental_pid_controller(self):
        controller = IncrementalPIDController(*self._get_params())
        self._test_pid_controller(controller, positive_direction=True)
        controller = IncrementalPIDController(*self._get_params())
        self._test_pid_controller(controller, positive_direction=False)

    def _create_feedforward_file(self) -> str:
        FILE_PATH = "test_feedforward_terms_file.csv"
        test_pwm_speeds = [
            (-1.0, -1.0),
            (-0.2, -0.2),
            (0.0, 0.0),
            (0.2, 0.2),
            (1.0, 1.0),
        ]
        with open(FILE_PATH, "w") as f:
            writer = csv.writer(f)
            writer.writerow(["pwm", "speed"])
            for pwm, speed in test_pwm_speeds:
                writer.writerow([pwm, speed])
        return FILE_PATH

    def test_feedforward_incremental_pid_controller(self):
        test_feedforward_terms_file = self._create_feedforward_file()
        controller = FeedforwardIncrementalPIDController(
            *self._get_params(),
            test_feedforward_terms_file,
            test_feedforward_terms_file,
        )
        # Testing feedforward search
        test_feedforward_search = [
            # speed, expected closest speed
            (-2, -1),
            (-1, -1),
            (-0.7, -0.2),
            (0.3, 0.2),
            (0.5, 0.2),
            (0.7, 0.2),
            (1, 0.2),
        ]
        for speed, expected_speed in test_feedforward_search:
            controller.store_commanded_speed((speed, speed))
            closest_smaller_speed = controller._find_closest_feedforward_pwms()
            assert np.allclose(expected_speed, closest_smaller_speed)
        self._test_pid_controller(controller, positive_direction=True)
        # controller = FeedforwardIncrementalPIDController(*self._get_params())
        # self._test_pid_controller(controller, positive_direction=False)
        try_remove_file(test_feedforward_terms_file)
