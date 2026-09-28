"""Sharp ADC-to-distance calibration supplied for this robot."""

from collections import deque
from statistics import median

from .setting import maze_setting as settings


class sharp_sensor:
    def __init__(self, ep_robot):
        self.ADC_MAX = settings.SHARP_ADC_MAX
        self.ADC_REFERENCE_V = settings.SHARP_ADC_REFERENCE_V
        self.VALID_MIN_CM = settings.SHARP_VALID_MIN_CM
        self.VALID_MAX_CM = settings.SHARP_VALID_MAX_CM
        self.REGRESSION_SLOPE = settings.SHARP_REGRESSION_SLOPE
        self.REGRESSION_INTERCEPT = settings.SHARP_REGRESSION_INTERCEPT
        self.DISTANCE_OFFSET_CM = settings.SHARP_DISTANCE_OFFSET_CM
        self.sensor_adaptor = ep_robot.sensor_adaptor
        self._distance_history = {}

    def get_distance(self, board, port):
        distance = self.read_sample(board, port)[2]
        key = (board, port)
        if distance is None:
            self._distance_history.pop(key, None)
            return None
        history = self._distance_history.setdefault(
            key, deque(maxlen=settings.SHARP_FILTER_WINDOW))
        history.append(distance)
        return median(history)

    def reset_filter(self):
        """Discard readings from a previous cell before starting alignment."""
        self._distance_history.clear()

    def read_sample(self, board, port):
        """Read raw ADC, voltage and unfiltered calibrated distance."""
        adc_value = self.sensor_adaptor.get_adc(id=board, port=port)
        voltage = self.adc_to_voltage(adc_value)
        return adc_value, voltage, self.voltage_to_distance_cm(voltage)

    def adc_to_voltage(self, adc_value):
        """Convert the Sensor Adapter's 10-bit ADC reading to volts."""
        if not 0 <= adc_value <= self.ADC_MAX:
            raise ValueError(f'ADC value must be in the range 0..{self.ADC_MAX}')
        return adc_value * self.ADC_REFERENCE_V / self.ADC_MAX

    def voltage_to_distance_cm(self, voltage):
        """Estimate distance with the regression, or None outside 4-30 cm."""
        if voltage <= self.REGRESSION_INTERCEPT:
            return None
        distance_cm = (
            self.REGRESSION_SLOPE / (voltage - self.REGRESSION_INTERCEPT)
            + self.DISTANCE_OFFSET_CM
        )
        if not self.VALID_MIN_CM <= distance_cm <= self.VALID_MAX_CM:
            return None
        return distance_cm
