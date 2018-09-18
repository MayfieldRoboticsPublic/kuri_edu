from .light_animation import LightAnimation


class PulseAnimation(LightAnimation):

    def __init__(self, framerate=10, color=(0, 0, 0)):
        super(LightAnimation, self).__init__(framerate, color)
        self._frame_index = 0

    def next(self):
        pass
