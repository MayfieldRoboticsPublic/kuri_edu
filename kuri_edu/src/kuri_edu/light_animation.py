import mobile_base


class LightAnimation(object):
    '''
    A light animation that plays a solid color for every frame.

    This can be used as a base class for other more complicated
    animations
    '''

    def __init__(self, framerate=10, color=(0, 0, 0)):
        '''
        Constructs a basic light animation that outputs a fixed color

        :param framerate: The framerate the animation is expected to be
        played back at, in hz

        :param color: A 3-Tuple of RGB color information, each pixel
        ranging from 0 to 255
        '''
        self._fr = framerate
        self._color = color

    def __iter__(self):
        return self

    def next(self):
        '''
        Gets the next frame of pixels and advances to the next frame.

        Overridden by more complex animations

        :return: An array of colors that can be passed to a
        mobile_base.ChestLightClient put_pixels method
        '''
        # All LEDs the same color:
        return [self._color] * mobile_base.ChestLightClient.NUM_LEDS

    def set_color(self, color):
        self._color = color

    def get_color(self):
        return self._color

    def get_framerate(self):
        return self._fr
