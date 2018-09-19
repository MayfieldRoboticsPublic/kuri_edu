from mobile_base import ChestLightClient as clc

from .light_animation import LightAnimation


class PulseAnimation(LightAnimation):
    '''
    Plays a pulsing animation where the center LED, mid-row LEDs, and
    outer LEDs are lit up in order
    '''
    FRAMES = [
        clc.LED_CENTER + [3, 6],  # Center-only is too dim.
        clc.LED_CENTER + clc.LED_MID_RING,
        clc.LED_CENTER + clc.LED_MID_RING + clc.LED_OUTER_RING,
        clc.LED_OUTER_RING,
    ]

    def __init__(self, framerate=10, color=(0, 0, 0)):
        super(PulseAnimation, self).__init__(framerate, color)

        self._current_frame = 0.0
        self._frame_increment = (
            float(len(self.FRAMES))
            / framerate
            / 1.25  # Time in seconds that the animation takes to complete
        )

    def next(self):
        on_range = self.FRAMES[int(self._current_frame)]

        self._current_frame += self._frame_increment
        self._current_frame %= len(self.FRAMES)

        return self._get_frame(on_range, self.get_color())

    @staticmethod
    def _get_frame(on_range, color):
        '''
        Generates a list of RGB pixels that can be sent to a
        mobile_base.ChestLightClient object

        :param on_range: A list of LED indicies that should be turned
        on for this frame of the animation

        :param color: The color of the 'on' LEDs

        :return: Pixels that can be sent to a mobile_base.ChestLightClient's
        put_pixels method
        '''
        arr = [(0,0,0)] * clc.NUM_LEDS
        for idx in on_range:
            arr[idx] = color
        return arr
