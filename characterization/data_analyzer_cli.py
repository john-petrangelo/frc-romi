# This program is a modified version of the analyzer from frc-characterization.
# The GUI elements have been removed and a CLI interface has been added.

import copy
from datetime import datetime
import json
import logging
import os
from enum import Enum

import control as ctrl
import frccontrol as frc_ctrl
import pint

import numpy as np
import statsmodels.api as sm

# Setup the logger.
logger = logging.getLogger("logger")
log_format = "%(asctime)s %(levelname)-8s: %(message)s"
logging.basicConfig(level=logging.INFO, format=log_format)

# These are the indices of data stored in the json file
TIME_COL = 0
BATTERY_COL = 1
AUTOSPEED_COL = 2
L_VOLTS_COL = 3
R_VOLTS_COL = 4
L_ENCODER_P_COL = 5
R_ENCODER_P_COL = 6
L_ENCODER_V_COL = 7
R_ENCODER_V_COL = 8
GYRO_ANGLE_COL = 9

# The are the indices of data returned from prepare_data function
PREPARED_TM_COL = 0
PREPARED_V_COL = 1
PREPARED_POS_COL = 2
PREPARED_VEL_COL = 3
PREPARED_ACC_COL = 4
PREPARED_COS_COL = 5

PREPARED_MAX_COL = PREPARED_ACC_COL

filename = "characterization-data20210117-2330.json"

class Tests(Enum):
    ARM = "Arm"
    ELEVATOR = "Elevator"
    DRIVETRAIN = "Drivetrain"
    SIMPLE_MOTOR = "Simple"


class Units(Enum):
    __ureg__ = pint.UnitRegistry()

    FEET = "Feet", __ureg__.foot
    METERS = "Meters", __ureg__.meter
    INCHES = "Inches", __ureg__.inch
    RADIANS = "Radians", __ureg__.radian
    DEGREES = "Degrees", __ureg__.degree
    ROTATIONS = "Rotations", __ureg__.revolution

    def __new__(cls, *values):
        obj = object.__new__(cls)
        # first value is canonical value
        obj._value_ = values[0]

        # ureg value
        obj.unit = values[1]
        return obj


JSON_DATA_KEYS = ["slow-forward", "slow-backward", "fast-forward", "fast-backward"]

subsets = [
    "Forward Left",
    "Backward Left",
    "Forward Right",
    "Backward Right",
    "All Combined",
    "Forward Combined",
    "Backward Combined",
]

presetChoices = {
    "Default",
    "WPILib (2020-)",
    "WPILib (Pre-2020)",
    "Talon FX",
    "Talon SRX (2020-)",
    "Talon SRX (Pre-2020)",
    "Spark MAX (brushless)",
    "Spark MAX (brushed)",
}

directions = {"Combined", "Forward", "Backward"}

loopTypes = {"Position", "Velocity"}


def is_valid(*a_tuple):
    for a in a_tuple:
        if a is None:
            return False
    return True


def calc_track_width(table):
    # Note that this assumes the gyro angle is not modded (i.e. on [0, +infinity)),
    # and that a positive angle travels in the counter-clockwise direction

    d_left = table[-1][R_ENCODER_P_COL] - table[0][R_ENCODER_P_COL]
    d_right = table[-1][L_ENCODER_P_COL] - table[0][L_ENCODER_P_COL]
    d_angle = table[-1][GYRO_ANGLE_COL] - table[0][GYRO_ANGLE_COL]

    if d_angle == 0:
        logger.error(
            "Error! Change in gyro angle was 0... Is your gyro set up correctly?",
        )
        return 0.0

    # The below comes from solving ω=(vr−vl)/2r for 2r
    # Absolute values used to ensure the calculated value is always positive
    # and to add robustness to sensor inversion
    diameter = (abs(d_left) + abs(d_right)) / abs(d_angle)

    return diameter


def is_rotation(units):
    return Units(units) in (Units.ROTATIONS, Units.RADIANS, Units.DEGREES)


def smooth_derivative(tm, value, n):
    """
    :param tm: time column
    :param value: Value to take the derivative of
    :param n: smoothing parameter
    """
    dlen = len(value)
    dt = tm[n:dlen] - tm[: (dlen - n)]
    x = (value[n:dlen] - value[: (dlen - n)]) / dt

    # pad to original length by adding zeros on either side
    return np.pad(
        x, (int(np.ceil(n / 2.0)), int(np.floor(n / 2.0))), mode="constant"
    )


def trim_step_test_data(data):
    # removes anything before the max acceleration
    max_accel_idx = np.argmax(np.abs(data[PREPARED_ACC_COL]))
    return data[:, max_accel_idx + 1:]


def compute_accel_drive(data, window):
    """
    Returned data columns correspond to PREPARED_*
    """

    # deal with incomplete data
    if len(data[TIME_COL]) < window * 2:
        logger.error(
            "Error! Not enough data points to compute acceleration. "
            + "Try running with a smaller window setting or a smaller threshold.",
        )
        return None

    # Compute left/right acceleration
    l_acc = smooth_derivative(data[TIME_COL], data[L_ENCODER_V_COL], window)
    r_acc = smooth_derivative(data[TIME_COL], data[R_ENCODER_V_COL], window)

    left = np.vstack(
        (
            data[TIME_COL],
            data[L_VOLTS_COL],
            data[L_ENCODER_P_COL],
            data[L_ENCODER_V_COL],
            l_acc,
        )
    )
    right = np.vstack(
        (
            data[TIME_COL],
            data[R_VOLTS_COL],
            data[R_ENCODER_P_COL],
            data[R_ENCODER_V_COL],
            r_acc,
        )
    )

    return left, right


def compute_accel(data, window):
    """
    Returned data columns correspond to PREPARED_*
    """

    # deal with incomplete data
    if len(data[TIME_COL]) < window * 2:
        logger.error(
            "Error! Not enough data points to compute acceleration. "
            + "Try running with a smaller window setting or a smaller threshold.",
        )
        return None

    # Compute left/right acceleration
    acc = smooth_derivative(data[TIME_COL], data[L_ENCODER_V_COL], window)

    dat = np.vstack(
        (
            data[TIME_COL],
            data[L_VOLTS_COL],
            data[L_ENCODER_P_COL],
            data[L_ENCODER_V_COL],
            acc,
        )
    )

    return dat


def ols(x1, x2, x3, y):
    """multivariate linear regression using ordinary least squares"""
    if x3:
        x = np.array((np.sign(x1), x1, x2, x3)).T
    else:
        x = np.array((np.sign(x1), x1, x2)).T
    model = sm.OLS(y, x)
    return model.fit()


def calc_fit(qu, step, test):
    vel = np.concatenate((qu[PREPARED_VEL_COL], step[PREPARED_VEL_COL]))
    accel = np.concatenate((qu[PREPARED_ACC_COL], step[PREPARED_ACC_COL]))
    volts = np.concatenate((qu[PREPARED_V_COL], step[PREPARED_V_COL]))

    test = Tests(test)
    if test == Tests.ELEVATOR:
        fit = ols(vel, accel, np.ones(vel.size), volts)
        ks, kv, ka, kg = fit.params
        r_square = fit.rsquared
        return kg, ks, kv, ka, r_square
    elif test == Tests.ARM:
        cos = np.concatenate((qu[PREPARED_COS_COL], step[PREPARED_COS_COL]))
        fit = ols(vel, accel, cos, volts)
        ks, kv, ka, kcos = fit.params
        r_square = fit.rsquared
        return ks, kv, ka, kcos, r_square
    else:
        fit = ols(vel, accel, None, volts)
        ks, kv, ka = fit.params
        r_square = fit.rsquared
    return ks, kv, ka, r_square


def calc_gains_pos(kv, ka, qp, qv, effort, period, position_delay):
    # If acceleration requires no effort, velocity becomes an input for position
    # control. We choose an appropriate model in this case to avoid numerical
    # instabilities in LQR.
    if ka > 1e-7:
        A = np.array([[0, 1], [0, -kv / ka]])
        B = np.array([[0], [1 / ka]])
        C = np.array([[1, 0]])
        D = np.array([[0]])

        q = [qp, qv]  # units and units/s acceptable errors
        r = [effort]  # V acceptable actuation effort
    else:
        A = np.array([[0]])
        B = np.array([[1]])
        C = np.array([[1]])
        D = np.array([[0]])

        q = [qp]  # units acceptable error
        r = [qv]  # units/s acceptable error
    sys = ctrl.ss(A, B, C, D)
    d_sys = sys.sample(period)

    # Assign Q and R matrices according to Bryson's rule [1]. The elements
    # of q and r are tunable by the user.
    #
    # [1] Bryson's rule in
    #     https://file.tavsys.net/control/state-space-guide.pdf
    Q = np.diag(1.0 / np.square(q))
    R = np.diag(1.0 / np.square(r))
    K = frc_ctrl.lqr(d_sys, Q, R)

    if position_delay > 0:
        # This corrects the gain to compensate for measurement delay, which
        # can be quite large as a result of filtering for some motor
        # controller and sensor combinations. Note that this will result in
        # an overly conservative (i.e. non-optimal) gain, because we need to
        # have a time-varying control gain to give the system an initial kick
        # in the right direction. The state will converge to zero and the
        # controller gain will converge to the steady-state one the tool outputs.
        #
        # See E.4.2 in
        #   https://file.tavsys.net/control/controls-engineering-in-frc.pdf
        delay_in_seconds = position_delay / 1000  # ms -> s
        K = K @ np.linalg.matrix_power(
            d_sys.A - d_sys.B @ K, round(delay_in_seconds / period)
        )

    # With the alternate model, `kp = kv * K[0, 0]` is used because the gain
    # produced by LQR is for velocity. We can use the feedforward equation
    # `u = kv * v` to convert velocity to voltage. `kd = 0` because velocity
    # was an input; we don't need feedback control to command it.
    if ka > 1e-7:
        kp = K[0, 0]
        kd = K[0, 1]
    else:
        kp = kv * K[0, 0]
        kd = 0

    return kp, kd


def calc_gains_vel(kv, ka, qv, effort, period, velocity_delay):

    # If acceleration for velocity control requires no effort, the feedback
    # control gains approach zero. We special-case it here because numerical
    # instabilities arise in LQR otherwise.
    if ka < 1e-7:
        return 0, 0

    A = np.array([[-kv / ka]])
    B = np.array([[1 / ka]])
    C = np.array([[1]])
    D = np.array([[0]])
    sys = ctrl.ss(A, B, C, D)
    dsys = sys.sample(period)

    # Assign Q and R matrices according to Bryson's rule [1]. The elements
    # of q and r are tunable by the user.
    #
    # [1] Bryson's rule in
    #     https://file.tavsys.net/control/state-space-guide.pdf
    q = [qv]  # units/s acceptable error
    r = [effort]  # V acceptable actuation effort
    Q = np.diag(1.0 / np.square(q))
    R = np.diag(1.0 / np.square(r))
    K = frc_ctrl.lqr(dsys, Q, R)

    if velocity_delay > 0:
        # This corrects the gain to compensate for measurement delay, which
        # can be quite large as a result of filtering for some motor
        # controller and sensor combinations. Note that this will result in
        # an overly conservative (i.e. non-optimal) gain, because we need to
        # have a time-varying control gain to give the system an initial kick
        # in the right direction. The state will converge to zero and the
        # controller gain will converge to the steady-state one the tool outputs.
        #
        # See E.4.2 in
        #   https://file.tavsys.net/control/controls-engineering-in-frc.pdf
        delay_in_seconds = velocity_delay / 1000  # ms -> s
        K = K @ np.linalg.matrix_power(
            dsys.A - dsys.B @ K, round(delay_in_seconds / period)
        )

    kp = K[0, 0]
    kd = 0

    return kp, kd


class Analyzer:
    def __init__(self, project_dir):

        self.window_size = 8
        self.project_path = project_dir
        self.motion_threshold = 0.2
        self.subset = "Forward Left"
        self.units = ""
        self.track_width = "N/A"

        self.qp = 1.0
        self.qv = 1.5
        self.max_effort = 7.0

        self.period = 0.02

        self.max_controller_output = 12.0

        self.controller_time_normalized = True

        self.measurement_delay = 0.0

        self.gearing = 1.0

        self.controller_type = "Onboard"

        self.encoder_epr = 4096

        self.has_follower = False

        self.follower_period = 0.01

        self.gain_units_preset = "Default"

        self.loop_type = "Velocity"

        # Feed forward results
        self.ks = 0.0
        self.kv = 0.0
        self.ka = 0.0
        self.kcos = 0.0
        self.r_square = 0.0

        # PID Feedback results
        self.kp = 0.0
        self.kd = 0.0
        self.kg = 0.0
        self.kcos = 0.0

        self.test = ""

        self.units_per_rot = 0.0

        self.convert_gains = False

        self.stored_data = None
        self.prepared_data = None

    def get_file(self):
        try:
            # TODO Filename should be a parameter
            with open(filename) as dataFile:
                data = json.load(dataFile)

            try:
                # Transform the data into a numpy array to make it easier to use
                # -> transpose it so we can deal with it in columns
                for k in JSON_DATA_KEYS:
                    data[k] = np.array(data[k]).transpose()

                self.stored_data = data
                logger.debug("Loaded Data")

                self.units = data["units"]
                self.test = data["test"]
                self.units_per_rot = float(data["unitsPerRotation"])
            except Exception as e:
                logger.error(
                    "Error! The structure of the data JSON was not recognized.\n"
                    + "Details\n"
                    + repr(e),
                )
                return
        except Exception as e:
            logger.error(
                "Error! The JSON file could not be loaded.\n" + "Details:\n" + repr(e))
            return

    def run_analysis(self):
        self.prepared_data = self.prepare_data(self.stored_data, window=self.window_size)

        if not self.prepared_data["Valid"]:
            return

        # Run the expected test type
        if Tests(self.test) == Tests.DRIVETRAIN:
            self.run_analysis_drive()
        elif Tests(self.test) == Tests.ELEVATOR:
            self.run_analysis_elevator()
        elif self.test == Tests.ARM:
            self.run_analysis_arm()
        elif Tests(self.test) == Tests.SIMPLE_MOTOR:
            self.run_analysis_simple()
        else:
            logger.error("Error! Unknown test type.")

        self.calc_gains()

    def run_analysis_drive(self):
        logger.debug("Running drive analysis")
        ks, kv, ka, r_square = calc_fit(
            *self.prepared_data[self.subset], self.test
        )

        self.ks = float("%.3g" % ks)
        self.kv = float("%.3g" % kv)
        self.ka = float("%.3g" % ka)
        self.r_square = float("%.3g" % r_square)

        if "track-width" in self.stored_data:
            self.track_width = calc_track_width(self.stored_data["track-width"])
        else:
            self.track_width = "N/A"

    def run_analysis_elevator(self):
        logger.debug("Running elevator analysis")
        kg, kfr, kv, ka, rsquare = calc_fit(
            *self.prepared_data[self.subset], self.test
        )

        self.kg = float("%.3g" % kg)
        self.ks = float("%.3g" % kfr)
        self.kv = float("%.3g" % kv)
        self.ka = float("%.3g" % ka)
        self.r_square = float("%.3g" % rsquare)

    def run_analysis_arm(self):
        logger.debug("Running arm analysis")
        ks, kv, ka, kcos, rsquare = calc_fit(
            *self.prepared_data[self.subset], self.test
        )

        self.ks = float("%.3g" % ks)
        self.kv = float("%.3g" % kv)
        self.ka = float("%.3g" % ka)
        self.kcos = float("%.3g" % kcos)
        self.r_square = float("%.3g" % rsquare)

    def run_analysis_simple(self):
        logger.debug("Running simple analysis")
        ks, kv, ka, rsquare = calc_fit(
            *self.prepared_data[self.subset], self.test
        )

        self.ks = float("%.3g" % ks)
        self.kv = float("%.3g" % kv)
        self.ka = float("%.3g" % ka)
        self.r_square = float("%.3g" % rsquare)

    def calc_gains(self):
        logger.debug("Calculating PID gains")
        period = (
            self.period
            if not self.has_follower
            else self.follower_period
        )

        if self.loop_type == "Position":
            kp, kd = calc_gains_pos(
                self.kv,
                self.ka,
                self.qp,
                self.qv,
                self.max_effort,
                period,
                self.measurement_delay,
            )
        else:
            kp, kd = calc_gains_vel(
                self.kv,
                self.ka,
                self.qv,
                self.max_effort,
                period,
                self.measurement_delay,
            )

        # Scale gains to output
        kp = kp / 12 * self.max_controller_output
        kd = kd / 12 * self.max_controller_output

        # Rescale kD if not time-normalized
        if not self.controller_time_normalized:
            kd = kd / self.period

        # Get correct conversion factor for rotations
        units = Units(self.units)
        if is_rotation(units.value):
            rotation = (1 * units.ROTATIONS.unit).to(units.unit)
        else:
            rotation = self.units_per_rot

        # Convert to controller-native units if desired
        if self.convert_gains:
            if self.controller_type == "Talon":
                kp = kp * rotation / (self.encoder_epr * self.gearing)
                kd = kd * rotation / (self.encoder_epr * self.gearing)
                if self.loop_type == "Velocity":
                    kp = kp * 10
            if self.controller_type == "Spark":
                kp = kp / self.gearing
                kd = kd / self.gearing
                if self.loop_type == "Velocity":
                    kp = kp / 60

        self.kp = float("%.3g" % kp)
        self.kd = float("%.3g" % kd)

    # TODO This is never called, look for presetGains in original.
    def preset_gains(self):
        def set_measurement_delay(delay):
            self.measurement_delay = 0 if self.loop_type == "Position" else delay

        # A number of motor controllers use moving average filters; these are types of FIR filters.
        # A moving average filter with a window size of N is a FIR filter with N taps.
        # The average delay (in taps) of an arbitrary FIR filter with N taps is (N-1)/2.
        # All of the delays below assume that 1 T takes 1 ms.
        #
        # Proof:
        # N taps with delays of 0 .. N - 1 T
        #
        # average delay = (sum 0 .. N - 1) / N T
        # = (sum 1 .. N - 1) / N T
        #
        # note: sum 1 .. n = n(n + 1) / 2
        #
        # = (N - 1)((N - 1) + 1) / (2N) T
        # = (N - 1)N / (2N) T
        # = (N - 1)/2 T

        if self.gain_units_preset == "WPILib (2020-)":
            self.max_controller_output = 12
            self.period = 0.02
            self.controller_time_normalized = True
            self.controller_type = "Onboard"
            # Note that the user will need to remember to set this if the onboard controller is getting
            # delayed measurements.
            set_measurement_delay(0)
        elif self.gain_units_preset == "WPILib (Pre-2020)":
            self.max_controller_output = 1
            self.period = 0.05
            self.controller_time_normalized = False
            self.controller_type = "Onboard"
            # Note that the user will need to remember to set this if the onboard controller is getting
            # delayed measurements.
            set_measurement_delay(0)
        elif self.gain_units_preset == "Talon FX":
            self.max_controller_output = 1
            self.period = 0.001
            self.controller_time_normalized = True
            self.controller_type = "Talon"
            # https://phoenix-documentation.readthedocs.io/en/latest/ch14 MCSensor.html#changing-velocity-measurement-parameters
            # 100 ms sampling period + a moving average window size of 64
            # (i.e. a 64-tap FIR) = 100/2 ms + (64-1)/2 ms = 81.5 ms.
            # See above for more info on moving average delays.
            set_measurement_delay(81.5)
        elif self.gain_units_preset == "Talon SRX (2020-)":
            self.max_controller_output = 1
            self.period = 0.001
            self.controller_time_normalized = True
            self.controller_type = "Talon"
            # https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#changing-velocity-measurement-parameters
            # 100 ms sampling period + a moving average window size of 64
            # (i.e. a 64-tap FIR) = 100/2 ms + (64-1)/2 ms = 81.5 ms.
            # See above for more info on moving average delays.
            set_measurement_delay(81.5)
        elif self.gain_units_preset == "Talon SRX (Pre-2020)":
            self.max_controller_output = 1023
            self.period = 0.001
            self.controller_time_normalized = False
            self.controller_type = "Talon"
            # https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#changing-velocity-measurement-parameters
            # 100 ms sampling period + a moving average window size of 64 (i.e. a 64-tap FIR) = 100/2 ms + (64-1)/2 ms = 81.5 ms.
            # See above for more info on moving average delays.
            set_measurement_delay(81.5)
        elif self.gain_units_preset == "Spark MAX (brushless)":
            self.max_controller_output = 1
            self.period.set = 0.001
            self.controller_time_normalized = False
            self.controller_type = "Spark"
            # According to a Rev employee on the FRC Discord the window size is 40 so delay = (40-1)/2 ms = 19.5 ms.
            # See above for more info on moving average delays.
            set_measurement_delay(19.5)
        elif self.gain_units_preset == "Spark MAX (brushed)":
            self.max_controller_output = 1
            self.period = 0.001
            self.controller_time_normalized = False
            self.controller_type = "Spark"
            # https://www.revrobotics.com/content/sw/max/sw-docs/cpp/classrev_1_1_c_a_n_encoder.html#a7e6ce792bc0c0558fb944771df572e6a
            # 64-tap FIR = (64-1)/2 ms = 31.5 ms delay.
            # See above for more info on moving average delays.
            set_measurement_delay(31.5)
        else:
            self.max_controller_output = 12
            self.period = 0.02
            self.controller_time_normalized = True
            self.controller_type = "Onboard"
            set_measurement_delay(0)

        if "Talon" in self.gain_units_preset or "Spark" in self.gain_units_preset:
            self.convert_gains = True
        else:
            self.convert_gains = False

    # From 449's R script (note: R is 1-indexed)

    # Create one for one sided and one for 2 sided
    def trim_quasi_test_data(self, data):
        abs_data = np.abs(data)
        test = Tests(self.test)
        if test == Tests.DRIVETRAIN:
            truth = np.all(
                [
                    abs_data[L_ENCODER_V_COL] > self.motion_threshold,
                    abs_data[L_VOLTS_COL] > 0,
                    abs_data[R_ENCODER_V_COL] > self.motion_threshold,
                    abs_data[R_VOLTS_COL] > 0,
                ],
                axis=0,
            )
        else:
            truth = np.all(
                [
                    abs_data[L_ENCODER_V_COL] > self.motion_threshold,
                    abs_data[L_VOLTS_COL] > 0,
                ],
                axis=0,
            )

        temp = data.transpose()[truth].transpose()

        if temp[TIME_COL].size == 0:
            logger.error(
                "Error! No data in quasistatic test is above motion threshold. "
                + "Try running with a smaller motion threshold (use --motion_threshold) "
                + "and make sure your encoder is reporting correctly!",
            )
            return None
        else:
            return temp

    def prepare_data_drivetrain(self, data, window):
        """
        Firstly, data should be 'trimmed' to exclude any data points at which the
        robot was not being commanded to do anything.

        Secondly, robot acceleration should be calculated from robot velocity and time.
        We have found it effective to do this by taking the slope of the secant line
        of velocity over a 60ms (3 standard loop iterations) window.

        Thirdly, data from the quasi-static test should be trimmed to exclude the
        initial period in which the robot is not moving due to static friction
        Fourthly, data from the step-voltage acceleration tests must be trimmed to
        remove the initial 'ramp-up' period that exists due to motor inductance; this
        can be done by simply removing all data points before maximum acceleration is
        reached.

        Finally, the data can be analyzed: pool your trimmed data into four data sets
        - one for each side of the robot (left or right) and each direction (forwards
        or backwards).

        For each set, run a linear regression of voltage seen at the motor
        (or battery voltage if you do not have Talon SRXs) versus velocity and
        acceleration.

        Voltage should be in units of volts, velocity in units of feet per second,
        and acceleration in units of feet per second squared.

        Each data pool will then yield three parameters -
        intercept, Kv (the regression coefficient of velocity), and Ka (the regression
        coefficient of acceleration).
        """
        # ensure voltage sign matches velocity sign and converts rotation measurements into proper units
        for x in JSON_DATA_KEYS:
            data[x][L_VOLTS_COL] = np.copysign(
                data[x][L_VOLTS_COL], data[x][L_ENCODER_V_COL]
            )
            data[x][R_VOLTS_COL] = np.copysign(
                data[x][R_VOLTS_COL], data[x][R_ENCODER_V_COL]
            )
            data[x][R_ENCODER_V_COL] = (
                np.array(data[x][R_ENCODER_V_COL]) * self.units_per_rot
            ).tolist()
            data[x][L_ENCODER_V_COL] = (
                np.array(data[x][L_ENCODER_V_COL]) * self.units_per_rot
            ).tolist()
            data[x][R_ENCODER_P_COL] = (
                np.array(data[x][R_ENCODER_V_COL]) * self.units_per_rot
            ).tolist()
            data[x][L_ENCODER_P_COL] = (
                np.array(data[x][L_ENCODER_V_COL]) * self.units_per_rot
            ).tolist()

        # trim quasi data before computing acceleration
        sf_trim = self.trim_quasi_test_data(data["slow-forward"])
        sb_trim = self.trim_quasi_test_data(data["slow-backward"])

        if sf_trim is None or sb_trim is None:
            return [None] * 8

        sf_l, sf_r = compute_accel_drive(sf_trim, window)
        sb_l, sb_r = compute_accel_drive(sb_trim, window)

        if sf_l is None or sf_r is None or sb_l is None or sb_r is None:
            return [None] * 8

        # trim step data after computing acceleration
        ff_l, ff_r = compute_accel_drive(data["fast-forward"], window)
        fb_l, fb_r = compute_accel_drive(data["fast-backward"], window)

        if ff_l is None or ff_r is None or fb_l is None or fb_r is None:
            return [None] * 8

        ff_l = trim_step_test_data(ff_l)
        ff_r = trim_step_test_data(ff_r)
        fb_l = trim_step_test_data(fb_l)
        fb_r = trim_step_test_data(fb_r)

        dataset = {
            "Forward Left": [sf_l, ff_l],
            "Forward Right": [sf_r, ff_r],
            "Backward Left": [sb_l, fb_l],
            "Backward Right": [sb_r, fb_r],
            "Forward Combined": [
                np.concatenate((sf_l, sf_r), axis=1),
                np.concatenate((ff_l, ff_r), axis=1),
            ],
            "Backward Combined": [
                np.concatenate((sb_l, sb_r), axis=1),
                np.concatenate((fb_l, fb_r), axis=1),
            ],
            "All Combined": [
                np.concatenate((sf_l, sb_l, sf_r, sb_r), axis=1),
                np.concatenate((ff_l, fb_l, ff_r, ff_r), axis=1),
            ],
            "Valid": is_valid(sf_l, sb_l, ff_l, fb_l, sf_r, sb_r, ff_r, fb_r),
        }

        return dataset

    def prepare_data(self, og_data, window):
        """
        Firstly, data should be 'trimmed' to exclude any data points at which the
        robot was not being commanded to do anything.

        Secondly, robot acceleration should be calculated from robot velocity and time.
        We have found it effective to do this by taking the slope of the secant line
        of velocity over a 60ms (3 standard loop iterations) window.

        Thirdly, data from the quasi-static test should be trimmed to exclude the
        initial period in which the robot is not moving due to static friction
        Fourthly, data from the step-voltage acceleration tests must be trimmed to
        remove the initial 'ramp-up' period that exists due to motor inductance; this
        can be done by simply removing all data points before maximum acceleration is
        reached.

        Finally, the data can be analyzed: pool your trimmed data into four data sets
        - one for each side of the robot (left or right) and each direction (forwards
        or backwards).

        For each set, run a linear regression of voltage seen at the motor
        (or battery voltage if you do not have Talon SRXs) versus velocity and
        acceleration.

        Voltage should be in units of volts, velocity in units of feet per second,
        and acceleration in units of feet per second squared.

        Each data pool will then yield three parameters -
        intercept, Kv (the regression coefficient of velocity), and Ka (the regression
        coefficient of acceleration).
        """
        logger.debug("Preparing data")

        # create a copy so original data doesn't get changed
        data = copy.deepcopy(og_data)

        test = Tests(self.test)
        if test == Tests.DRIVETRAIN:
            return self.prepare_data_drivetrain(data, window)
        else:
            # Ensure voltage points in same direction as velocity
            for x in JSON_DATA_KEYS:
                data[x][L_VOLTS_COL] = np.copysign(
                    data[x][L_VOLTS_COL], data[x][L_ENCODER_V_COL]
                )
                data[x][L_ENCODER_V_COL] = (
                    np.array(data[x][L_ENCODER_V_COL]) * self.units_per_rot
                ).tolist()
                data[x][L_ENCODER_P_COL] = (
                    np.array(data[x][L_ENCODER_V_COL]) * self.units_per_rot
                ).tolist()

            # trim quasi data before computing acceleration
            sf_trim = self.trim_quasi_test_data(data["slow-forward"])
            sb_trim = self.trim_quasi_test_data(data["slow-backward"])

            if sf_trim is None or sb_trim is None:
                return None, None, None, None

            sf = compute_accel(sf_trim, window)
            sb = compute_accel(sb_trim, window)

            if sf is None or sb is None:
                return None, None, None, None

            # trim step data after computing acceleration
            ff = compute_accel(data["fast-forward"], window)
            fb = compute_accel(data["fast-backward"], window)

            if ff is None or fb is None:
                return None, None, None, None

            ff = trim_step_test_data(ff)
            fb = trim_step_test_data(fb)

            dataset = {
                "Forward": [sf, ff],
                "Backward": [sb, fb],
                "Combined": [
                    np.concatenate((sf, sb), axis=1),
                    np.concatenate((ff, fb), axis=1),
                ],
                "Valid": is_valid(sf, sb, ff, fb),
            }
            return dataset

    def report_parameters(self):
        subset = self.subset.split(" ")
        return (
            f"         +--------------+-----------------+-----------------+-----------------+------------------+\n"
            f"{subset[0]:8} | Feed forward |  kS: {self.ks:9.3g}  |  kV: {self.kv:9.3g}  |  kA: {self.ka:9.3g}  |  R^2: {self.r_square:9.3g}  |\n"
            f"{subset[1]:8} | PID Feedback |  kP: {self.kp:9.3g}  |  kD: {self.kd:9.3g}  |                 |                  |\n"
            f"         +--------------+-----------------+-----------------+-----------------+------------------+\n"
        )


def main(project_dir):
    analyzer = Analyzer(project_dir)
    analyzer.get_file()

    # Print report header
    print(f"Robot characterization report for {analyzer.test}")
    print(datetime.now().strftime("Analysis run on %b %d, %Y at %H:%M:%S"))
    print(f"Using data from {filename}")
    print()

    # Print results for each test subset
    for subset in subsets:
        analyzer.subset = subset
        analyzer.run_analysis()
        print(analyzer.report_parameters())

    # Print results for track width
    print(f"Track width: {analyzer.track_width:5.3g} {analyzer.units:10}    (TODO not quite right yet...)")


if __name__ == "__main__":
    main(os.getcwd())

# TODO Add CLI options
# --debug       enable debug logging
# -f filename   name of data file to read
