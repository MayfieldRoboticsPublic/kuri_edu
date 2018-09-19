from kuri_edu import ChestLedController

import maytest


class TestChestLightController(maytest.TestBase):

    def test_low_battery(self):
        # Check to make sure low battery (below 20%) is all 'red'
        expected_color = (255, 0, 0)

        self.assertEquals(
            ChestLedController._calculate_led_color(0),
            expected_color
        )

        self.assertEquals(
            ChestLedController._calculate_led_color(10),
            expected_color
        )

        self.assertEquals(
            ChestLedController._calculate_led_color(20),
            expected_color
        )

    def test_full_battery(self):
        expected_color = (0, 255, 0)  # All green

        self.assertEquals(
            ChestLedController._calculate_led_color(100),
            expected_color
        )

    def test_in_between(self):

        # 60 is the 'middle' color, because 20% and below is all 'red'
        # we only interpolate colors between 20% and 100%
        color = ChestLedController._calculate_led_color(60)

        self.assertEquals(color[0], 127)
        self.assertEquals(color[1], 127)
