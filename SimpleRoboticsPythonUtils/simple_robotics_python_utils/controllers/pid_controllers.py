#!/usr/bin/env python3

from collections import deque, namedtuple
import numpy as np
from typing import Tuple, Deque, Callable, List
import os
import csv
import bisect

PIDParams = namedtuple("PIDParams", ["kp", "ki", "kd"])

NUM_ERRORS = 2
MIN_PWM = -1.0
MAX_PWM = 1.0


class BasePIDController:
    """Controller Base Class for both motors using PID controller"""

    def __init__(self, left_params: PIDParams, right_params: PIDParams):
        self.kp = np.array((left_params.kp, right_params.kp))
        self.ki = np.array((left_params.ki, right_params.ki))
        self.kd = np.array((left_params.kd, right_params.kd))
        self.stop_getting_wheel_vel_updates()
        self.stop_taking_commands()

    ######################################### Wheel Vel Update Functions #########################################
    def _reset_pid_params(self):
        self.errors: Deque[np.ndarray] = deque([np.zeros(2) for _ in range(NUM_ERRORS)], maxlen=NUM_ERRORS)
        self.last_pwm: np.ndarray = np.zeros(2)

    def store_speed(self, speed: Tuple[float, float]) -> None:
        """Callback for hearing actual speeds

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.motor_speeds: np.ndarray = np.asarray(speed)

    def start_getting_wheel_vel_updates(self):
        """Set flag to True. Relevant params have been reset already"""
        self.getting_speed_updates = True

    def stop_getting_wheel_vel_updates(self):
        """Function that resets current speeds and errors and flag."""
        self.getting_speed_updates = False
        self.motor_speeds = np.array((0.0, 0.0))
        self._reset_pid_params()

    ######################################### Commanded Wheel Vel Functions #########################################
    def store_commanded_speed(self, speed: Tuple[float, float]) -> None:
        """Callback for hearing commanded speed

        Args:
            speed (Tuple[float, float]): motor speed
        """
        self.desired_speeds = np.asarray(speed)

    def start_taking_commands(self):
        """Set flag to True. Relevant params have been reset already"""
        self.getting_commanded_wheel_vel = True

    def stop_taking_commands(self):
        """Function that resets current speeds and pwm and flag."""
        self.desired_speeds = np.array((0.0, 0.0))
        self.getting_commanded_wheel_vel = False

    def get_pwms(self) -> Tuple[float, float]:
        """High level function to calculate current PWM.

        Returns:
            Tuple[float, float]: PWM for both motors
        """
        if self.getting_speed_updates and self.getting_commanded_wheel_vel:
            # e[k] = desired_speeds[k] - motor_speeds[k]
            e = np.asarray(self.desired_speeds) - self.motor_speeds
            current_pwm = self.calc_pid(e, self.errors, self.kp, self.ki, self.kd, self.last_pwm)
            self.errors.appendleft(e)
            self.last_pwm = current_pwm
            return tuple(current_pwm)
        else:
            return (0.0, 0.0)

    def get_actual_speeds(self) -> Tuple[float, float]:
        """Small function to get the two motor speeds"""
        return tuple(self.motor_speeds)

    def calc_pid(
        self,
        e: np.ndarray,
        past_errors: Deque[np.ndarray],
        kp: np.ndarray,
        ki: np.ndarray,
        kd: np.ndarray,
        last_pwm: np.ndarray,
    ) -> np.ndarray:
        raise NotImplementedError("Calc PID must be implemented in child class")


class IncrementalPIDController(BasePIDController):
    """Controller for both motors using incremental PID controller"""

    def calc_pid(
        self,
        e: np.ndarray,
        past_errors: Deque[np.ndarray],
        kp: np.ndarray,
        ki: np.ndarray,
        kd: np.ndarray,
        last_pwm: np.ndarray,
    ) -> np.ndarray:
        """Util function to calculate the current pwm value.

        Args:
            e (np.ndarray): set_point - current_value
            past_errors (np.ndarray): e[k-1], e[k-2]
            kp (np.ndarray): 2-array for both motors
            ki (np.ndarray): 2-array for both motors
            kd (np.ndarray): 2-array for both motors
            last_pwm (np.ndarray): last pwm signal

        Returns:
            np.ndarray: current pwm
        """
        # u[k] = kp * (e[k] - e[k-1]) + ki * e[k] + kd * (e[k] - 2 * e[k-1] + e[k-2])
        u = kp * (e - past_errors[0]) + ki * e + kd * (e - 2 * past_errors[0] + past_errors[1])
        # TODO
        # print(f'p: {kp * (e - past_errors[0])}, i: {ki * e}, d: {kd * (e - 2 * past_errors[0] + past_errors[1])}, last_pwm: {last_pwm}')
        current_pwm = u + last_pwm
        # (-1, 1) means Bi-directional
        current_pwm = np.clip(current_pwm, MIN_PWM, MAX_PWM)
        return current_pwm


class RegularDiscretePIDController(BasePIDController):
    """Notes
    Experiment observations:
        - It's quite smooth, but steady state error is hard to kill.
        - You may want to jack up Kp, but that could lead to some nasty instabilities
    """

    def __init__(self, left_params: PIDParams, right_params: PIDParams):
        global NUM_ERRORS
        NUM_ERRORS = 5
        super().__init__(left_params, right_params)

    def calc_pid(
        self,
        e: np.ndarray,
        past_errors: Deque[np.ndarray],
        kp: np.ndarray,
        ki: np.ndarray,
        kd: np.ndarray,
        last_pwm: np.ndarray,
    ) -> np.ndarray:
        """Util function to calculate the current pwm value.

        Args:
            e (np.ndarray): set_point - current_value
            past_errors (np.ndarray): e[k-1], e[k-2]
            kp (np.ndarray): 2-array for both motors
            ki (np.ndarray): 2-array for both motors
            kd (np.ndarray): 2-array for both motors
            last_pwm (np.ndarray): last pwm signal

        Returns:
            np.ndarray: current pwm
        """
        # u[k] = kp * (e[k] - e[k-1]) + ki * e[k] + kd * (e[k] - 2 * e[k-1] + e[k-2])
        u = kp * e + ki * (sum(self.errors)) / len(self.errors) + kd * (e - past_errors[0])
        # # TODO
        # print(
        #     f"p: {kp * e}, i: {ki * (e + past_errors[0] + past_errors[1])}, d: {kd * (e  - past_errors[0])}"
        # )
        current_pwm = u
        # (-1, 1) means Bi-directional
        current_pwm = np.clip(current_pwm, MIN_PWM, MAX_PWM)
        return current_pwm


class FeedforwardIncrementalPIDController(IncrementalPIDController):
    LEFT_FEEDFOWARD_TERMS_FILE = "LEFT_FEEDFORWARD_TERMS.csv"
    RIGHT_FEEDFOWARD_TERMS_FILE = "RIGHT_FEEDFORWARD_TERMS.csv"
    __slots__ = (
        "need_to_record_feedforward_terms",
        "feedforward_terms",
        "last_feedforward_pwm",
    )

    """
    1. YOU SHOULD DEFINITELY TRY INCREMENTAL PID FIRST, chances are, the feedforward control is worse.
        - Performance of the feedforward controller greatly depends on how far the feedforward terms are
        - unfortunately, the motors speeds are dependent on battery voltage. So the feedforward terms can divert by quite a bit
        - If the model varies quite a bit from the feedforward terms, DO NOT USE THIS
    2. Recording is in the hands of the tuner.
        - So the tuner provides the full feedforward file path
            - But this class provides the file name, for reference
        - This class only provides a record tool function. That should be used as a callback for motor outputs
            - The tool function writes (pwm, actual speed) to the file
    3. Feedforward PID Design Notes
        - If your last pwm already containts the feedforward term, you are essentially accumulating feedforward term. So, do not include feedforward as part of your PID output
    """

    def __init__(
        self,
        left_params: PIDParams,
        right_params: PIDParams,
        left_feedforward_terms_file: str,
        right_feedforward_terms_file: str,
    ):
        super().__init__(left_params, right_params)

        self._load_feedforward_terms(left_feedforward_terms_file, right_feedforward_terms_file)

    def _load_feedforward_terms(self, left_feedforward_terms_file, right_feedforward_terms_file):
        # Build
        def _load_single_feedforward_terms(
            feedforward_terms_file: str,
        ) -> Tuple[List[float], List[float]]:
            with open(feedforward_terms_file, "r") as f:
                reader = csv.reader(f)
                feedforward_terms = []
                for row in reader:
                    try:
                        feedforward_terms.append((float(row[0]), float(row[1])))
                    except ValueError:
                        # will get this at header
                        pass
            # Sort the feedforward terms
            sorted(feedforward_terms, key=lambda x: x[0])
            feedforward_pwms = [x[0] for x in feedforward_terms]
            feedforward_speeds = [x[1] for x in feedforward_terms]
            return feedforward_pwms, feedforward_speeds

        (
            self.left_feedforward_pwms,
            self.left_feedforward_speeds,
        ) = _load_single_feedforward_terms(left_feedforward_terms_file)
        (
            self.right_feedforward_pwms,
            self.right_feedforward_speeds,
        ) = _load_single_feedforward_terms(right_feedforward_terms_file)

    ####################################################################
    # Reset more params
    ####################################################################
    def _reset_for_new_feedforward_terms(self):
        self.last_feedforward_pwm = 0

    def stop_taking_commands(self):
        super().stop_taking_commands()
        self._reset_for_new_feedforward_terms()

    ####################################################################
    # feedforward
    ####################################################################

    def _find_closest_feedforward_pwms(self) -> np.ndarray:
        # Use binary search for self.desired_speeds
        # TODO: one potential improvement is to use interpolation, or return the NEAREST smaller, or larger term
        def _binary_search_for_index(feedforward_pwms: List[Tuple[float, float]], desired_speed: float) -> int:
            if desired_speed < 0:
                next_smaller_num_offset = 0
            else:
                next_smaller_num_offset = -1
            index_closest_smaller = bisect.bisect_left(feedforward_pwms, self.desired_speeds[0]) + next_smaller_num_offset
            index_closest_smaller = max(0, index_closest_smaller)
            index_closest_smaller = min(index_closest_smaller, len(feedforward_pwms))
            return index_closest_smaller

        return np.array(
            [
                self.left_feedforward_speeds[_binary_search_for_index(self.left_feedforward_pwms, self.desired_speeds[0])],
                self.right_feedforward_speeds[_binary_search_for_index(self.right_feedforward_pwms, self.desired_speeds[1])],
            ]
        )

    def get_pwms(self) -> Tuple[float, float]:
        """
        High level function to calculate current PWM.
        When to reset: a new feedforward term is selected.
        Why reset the PID controller:
        - The current setpoint might introduce huge dips /overshoot at the beginning, since the are determined by previous errors
        - Need:
            - self.last_feedforward
            - upon new feedforward
                - e = 0 (this only buys you a few milliseconds)
                - reset self.last_error, self.last_pwm,

        Returns:
            Tuple[float, float]: PWM for both motors
        """
        if self.getting_speed_updates and self.getting_commanded_wheel_vel:
            # e[k] = desired_speeds[k] - motor_speeds[k]
            feedforward_term = self._find_closest_feedforward_pwms()
            if np.allclose(feedforward_term, self.last_feedforward_pwm):
                e = np.zeros(2)
                self._reset_for_new_feedforward_terms()
                self._reset_pid_params()
            else:
                e = np.asarray(self.desired_speeds) - self.motor_speeds

            incremental_pid_pwm = self.calc_pid(e, self.errors, self.kp, self.ki, self.kd, self.last_pwm)
            pwm_output = incremental_pid_pwm + feedforward_term
            pwm_output = np.clip(pwm_output, MIN_PWM, MAX_PWM)
            self.errors.appendleft(e)
            # IMPORTANT: self.last_pwm is actually incremental_pid_pwm ONLY!
            self.last_pwm = incremental_pid_pwm
            # print(f'pure incremental term: {incremental_pid_pwm}, feedforward term: {self._find_closest_feedforward_pwms()}')
            return tuple(pwm_output)
        else:
            return (0.0, 0.0)
