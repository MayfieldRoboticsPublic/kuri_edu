from kuri_edu import PulseAnimation

import mobile_base

import maytest


class TestPulseAnimation(maytest.TestBase):

    def setUp(self):
        super(TestPulseAnimation, self).setUp()
        self.dut = PulseAnimation()

    def test_red(self):
        self.dut.set_color((255, 0, 0))
        self.check_color_output(
            self.dut,
            (255, 0, 0)
        )

    def test_green(self):
        self.dut.set_color((0, 255, 0))
        self.check_color_output(
            self.dut,
            (0, 255, 0)
        )

    def check_color_output(self, anim, expected_color):

        for _ in range(len(anim.FRAMES)):
            frame = anim.next()

            for px in frame:
                if px == (0, 0, 0):
                    continue
                else:
                    self.assertEqual(px, expected_color)
