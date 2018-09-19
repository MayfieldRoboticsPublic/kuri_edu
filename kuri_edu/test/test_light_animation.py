from kuri_edu import LightAnimation

import mobile_base

import maytest


class TestLightAnimation(maytest.TestBase):

    def test_anim_outputs_same_color(self):
        dut = LightAnimation(
            color=(255, 255, 255)
        )

        idx = 0

        # LightAnimation behaves like an iterator.  It should spit
        # out the same color forever.
        for frame in dut:
            if idx >= 1000:
                break

            # Each frame should be 15 pixels all containing the same color:
            self.assertEqual(
                [(255, 255, 255)] * mobile_base.ChestLightClient.NUM_LEDS,
                frame
            )

            idx += 1

    def test_get_framerate(self):
        for n in range(10):
            dut = LightAnimation(framerate=n)
            self.assertEquals(dut.get_framerate(), n)
