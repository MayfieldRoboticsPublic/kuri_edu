# from kuri_edu import BatteryMonitor
# 
# import mock
# 
# import mobile_base_driver.msg
# 
# import fakerospy
# import maytest
# 
# 
# class TestBatteryMonitor(maytest.TestBase):
# 
#     def setUp(self):
#         super(TestBatteryMonitor, self).setUp()
#         self.patch('kuri_edu.battery_monitor.rospy', fakerospy)
#         self.patch('mobile_base.chest_light_client.rospy', fakerospy)
# 
#         self._power_pub = fakerospy.Publisher(
#             'mobile_base/power',
#             mobile_base_driver.msg.Power
#         )
# 
#         self.dut = BatteryMonitor()
# 
#     def tearDown(self):
#         super(TestBatteryMonitor, self).tearDown()
#         self.dut.shutdown()
#         fakerospy.reset()
# 
#     def publish_power(self, pct):
#         msg = mobile_base_driver.msg.Power(
#             battery=mobile_base_driver.msg.BatteryCapacity(
#                 mAh=2500,
#                 pct=pct,
#                 rounded_pct=int(pct/5)*5
#             )
#         )
#         self._power_pub.publish(msg)
# 
#     def test_filter_duplicate_power_levels(self):
#         # Make sure we're not blasing colors down to the chest LEDs on every
#         # mobile_base/power message
# 
#         with mock.patch.object(self.dut._light_client, 'put_pixels') as m_put:
#             m_put.assert_not_called()
# 
#             self.publish_power(100)
#             m_put.assert_called_once()
# 
#             # Second message with the same rounded_pct should not trigger
#             # a chest LED write
#             self.publish_power(100)
#             m_put.assert_called_once()
# 
#             self.publish_power(50)
#             self.assertEquals(
#                 m_put.call_count,
#                 2
#             )
