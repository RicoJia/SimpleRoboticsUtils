#!/usr/bin/env python3

from collections import deque, namedtuple
import numpy as np
from typing import Tuple, Deque, Callable
import os

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
        self.errors: Deque[np.ndarray] = deque(
            [np.zeros(2) for _ in range(NUM_ERRORS)], maxlen=NUM_ERRORS
        )

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
        self.last_pwm: np.ndarray = np.zeros(2)
        self.getting_commanded_wheel_vel = False

    def get_pwms(self) -> Tuple[float, float]:
        """High level function to calculate current PWM.

        Returns:
            Tuple[float, float]: PWM for both motors
        """
        if self.getting_speed_updates and self.getting_commanded_wheel_vel:
            # e[k] = desired_speeds[k] - motor_speeds[k]
            e = np.asarray(self.desired_speeds) - self.motor_speeds
            current_pwm = self.calc_pid(
                e, self.errors, self.kp, self.ki, self.kd, self.last_pwm
            )
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
        u = (
            kp * (e - past_errors[0])
            + ki * e
            + kd * (e - 2 * past_errors[0] + past_errors[1])
        )
        current_pwm = u + last_pwm
        # (-1, 1) means Bi-directional
        current_pwm = np.clip(current_pwm, MIN_PWM, MAX_PWM)
        return current_pwm


class FeedforwardIncrementalPIDController(BasePIDController):
    LEFT_FEEDFOWARD_TERMS_FILE = "LEFT_FEEDFORWARD_TERMS.csv"
    RIGHT_FEEDFOWARD_TERMS_FILE = "RIGHT_FEEDFORWARD_TERMS.csv"
    __slots__ = ("need_to_record_feedforward_terms", "feedforward_terms")

    """
    1. Recording is in the hands of the tuner.
        - So the tuner provides the full feedforward file path
            - But this class provides the file name, for reference
        - This class only provides a record tool function. That should be used as a callback for motor outputs
            - The tool function writes (pwm, actual speed) to the file
    """

    def __init__(
        self,
        left_params: PIDParams,
        right_params: PIDParams,
        need_to_record_feedforward_terms: bool = False,
        record_func: Callable[[], None] = None,
    ):
        super().__init__(left_params, right_params)
        need_to_record_feedforward_terms = need_to_record_feedforward_terms
        if not need_to_record_feedforward_terms and not os.path.exists(
            FeedforwardIncrementalPIDController.FEEDFOWARD_TERMS_FILE
        ):
            need_to_record_feedforward_terms = (
                input("Would you like to record feedforward terms? [y/n]\n") == "y"
            )

        if need_to_record_feedforward_terms:
            if record_func is None:
                raise RuntimeError(
                    "Please provide a function to record feedforward terms"
                )
            self._record_feedforward_terms(record_func)

        self._load_feedforward_terms()

    def _load_feedforward_terms(self):
        pass

    def _record_feedforward_terms(self, record_func):
        # This need to subscribe to an external speed topics
        # 1. for each pwm value, pass the pwm into the record function, then get a list of
        # outputs out
        pass
