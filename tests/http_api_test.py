#!/usr/bin/python
# -*- coding: utf-8 -*-
import logging
import unittest

from lebai import LebaiRobot, LebaiScene


# n)umber
from lebai.lebai_http_service import LebaiHttpService


class Test(unittest.TestCase):
    """Test http_api_test.py"""
    ip = "192.168.3.227"

    @classmethod
    def setUpClass(cls):
        LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
        logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.http_service = LebaiHttpService(self.ip)
        self.robot = LebaiRobot(self.ip, True)

    def tearDown(self):
        pass

    def get_task_id(self):
        code = "print('test')"
        return self.http_service.execute_lua_code('任务名称', 1, 1, code)['id']

    def test_get_tasks(self):
        r = self.http_service.get_tasks(0, 10)
        logging.info(r)
        self.assertTrue(r)

    def test_execute_lua_code(self):
        code = "print('test')"
        r = self.http_service.execute_lua_code('任务名称', 1, 1, code)
        self.assertTrue(r['id'])

    def test_get_task(self):
        r = self.http_service.get_task(self.get_task_id())
        self.assertTrue(r)

    def test_robot_run(self):
        task = LebaiScene(self.ip, scene_id=10054)
        return task.run(1)

    def test_run_task(self):
        r = self.http_service.run_task(self.get_task_id(), 1, 1)
        self.assertTrue(r)
        pass

    def test_run_scene(self):
        r = self.http_service.run_task(10001, 1, 1)
        self.assertTrue(r)
        pass

    # 状态控制

    def test_start_sys(self):
        r = self.http_service.action("start_sys")

        self.assertGreater(r['task_id'], 0)
        pass

    def test_stop_sys(self):
        r = self.http_service.action("stop_sys")

        self.assertGreater(r['task_id'], 0)
        pass

    def test_powerdown(self):
        r = self.http_service.action("power_down")

        self.assertGreater(r['task_id'], 0)

    def test_stop(self):
        r = self.http_service.action("stop")

        self.assertGreater(r['task_id'], 0)
        pass

    def test_estop(self):
        r = self.http_service.action("estop")

        self.assertGreater(r['task_id'], 0)
        pass

    def test_teach_mode(self):
        r = self.http_service.action("teach_mode")

        self.assertGreater(r['task_id'], 0)
        self.http_service.action("end_teach_mode")
        pass

    def test_end_teach_mode(self):
        self.http_service.action("teach_mode")
        r = self.http_service.action("end_teach_mode")

        self.assertGreater(r['task_id'], 0)
        pass

    # stop
    # def test_pause_task(self):
    #     r = self.http_service.action("pause_task")
    #
    #     self.assertGreater(r['task_id'], 0)
    #     pass

    def test_resume_task(self):
        r = self.http_service.action("resume_task")

        self.assertIsNotNone(r)
        self.assertGreater(r['task_id'], 0)
        pass

    def test_wait(self):
        self.http_service.action("wait")
        pass

    # def test_wait_until(self):
    #     def is_started():
    #         return True
    #
    #     self.http_service.action("wait_until", {
    #         'fn': is_started
    #     })
    #     pass

    # stop
    # def test_stop_task(self):
    #     r = self.http_service.action("stop_task")
    #
    #     self.assertGreater(r['task_id'], 0)
    #     pass

    def test_get_task_status(self):
        r = self.http_service.action("get_task_status")
        self.assertIn('task_id', r)
        self.assertIn('task_status', r)
        pass

    # 参数配置

    def test_set_tcp(self):
        data = {
            "value": [1.002, 1.002, 1.002, 1.002, 1.002, 1.002]
        }
        r = self.http_service.action("set_tcp", data)
        self.assertTrue(r['result'] == 1)
        pass

    def test_get_tcp(self):
        r = self.http_service.action("get_tcp")
        self.assertTrue('value' in r)
        self.assertTrue(len(r['value']) == 6)
        pass

    def test_set_velocity_factor(self):
        r = self.http_service.action("set_velocity_factor")

        self.assertGreater(r['task_id'], 0)
        pass

    def test_get_velocity_factor(self):
        r = self.http_service.action("get_velocity_factor")
        self.assertTrue(0 <= r['value'] <= 100)
        pass

    def test_set_payload(self):
        data = {
            "mass": 12, "cog": [1, 2, 3]
        }
        r = self.http_service.action("set_payload", data)
        self.assertTrue(r['result'] == 1)
        pass

    def test_get_payload(self):
        r = self.http_service.action("get_payload")
        self.assertTrue('cog' in r)
        self.assertTrue('mass' in r)
        pass

    def test_set_payload_mass(self):
        data = {
            'value': 2.0
        }
        r = self.http_service.action("set_payload_mass", data)
        self.assertTrue(r['result'] == 1)
        pass

    def test_get_payload_mass(self):
        self.http_service.action("get_payload_mass")
        pass

    def test_set_payload_cog(self):
        data = {
            "value": [1, 2, 3]
        }
        r = self.http_service.action("set_payload_cog", data)
        self.assertTrue(r['result'] == 1)
        pass

    # def test_get_payload_cog(self):
    #     r = self.http_service.action("get_payload_cog")
    #     # todo: 待实现
    #     self.assertTrue(False)
    #     pass

    def test_set_gravity(self):
        data = {
            "value": [0, 0, -9.82]
        }
        r = self.http_service.action("set_gravity", data)
        self.assertTrue('result' in r)
        self.assertEqual(r['result'], 1)
        pass

    def test_get_gravity(self):
        data = {}
        r = self.http_service.action("get_gravity", data)
        self.assertTrue('gravity' in r)
        self.assertEqual(len(r['gravity']), 3)
        pass

    # 移动命令

    def test_movej(self):
        data = {
            "pose_to": [0, -0.7853981633974483, 1.5707963267948966, -0.7853981633974483, 1.5707963267948966, 0],
            "is_joint_angle": True, "acceleration": 1.23, "velocity": 1.23
        }
        r = self.http_service.action("movej", data)
        self.assertGreater(r['task_id'], 0)
        pass

    def test_movel(self):
        data = {
            "pose_to": [0.2, 0.5, 0.4, 0, 0, 1.57],
            "is_joint_angle": False,
            "acceleration": 1.0, "velocity": 0.2
        }
        r = self.http_service.action("movel", data)
        self.assertGreater(r['task_id'], 0)
        pass

    def test_movec(self):
        data = {
            'pos_via': [0.2, 0.5, 0.4, 0, 0, 1.57],
            'pos': {'j1': 0.1, 'j2': 0.2, 'j3': 0.2, 'j4': 0.3, 'j5': 0.1, 'j6': 0.2},
            'rad': 0,
            'a': 0,
            'v': 1,
            't': 0.2,
            'r': 0
        }
        r = self.http_service.action("movec", data)
        self.assertGreater(r['task_id'], 0)
        pass

    def test_stop_move(self):
        r = self.http_service.action("stop_move")
        self.assertGreater(r['task_id'], 0)
        pass

    def test_movej_until(self):
        # data = {}
        # self.http_service.action("movej_until", data)
        # 暂不支持
        pass

    def test_movel_until(self):
        # data = {}
        # self.http_service.action("movel_until", data)
        # 暂不支持
        pass

    def test_movec_until(self):
        # data = {}
        # self.http_service.action("movec_until", data)
        # 暂不支持
        pass

    def test_movej_until_rt(self):
        # data = {}
        # self.http_service.action("movej_until_rt", data)
        # 暂不支持
        pass

    def test_movel_until_rt(self):
        # data = {}
        # self.http_service.action("movel_until_rt", data)
        # 暂不支持
        pass

    def test_movec_until_rt(self):
        # data = {}
        # self.http_service.action("movec_until_rt", data)
        # 暂不支持
        pass

    # 状态数据

    def test_robot_data(self):
        #

        # data = {}
        # self.http_service.action("robot_data", data)
        pass

    def test_get_robot_mode(self):
        r = self.http_service.action("get_robot_mode")
        self.assertIn('robot_mode', r)
        self.assertIn('velocity_factor', r)
        self.assertIn('actual_joint', r)
        self.assertIn('target_joint', r)
        self.assertIn('actual_tcp_pose', r)
        self.assertIn('target_tcp_pose', r)
        pass

    def test_get_actual_joint_positions(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_actual_joint_positions", data)
        pass

    def test_get_actual_joint_speeds(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_actual_joint_speeds", data)
        pass

    def test_get_target_joint_positions(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_target_joint_positions", data)
        pass

    def test_get_target_joint_speeds(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_target_joint_speeds", data)
        pass

    def test_get_actual_tcp_pose(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_actual_tcp_pose", data)
        pass

    def test_get_target_tcp_pose(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_target_tcp_pose", data)
        pass

    def test_get_joint_torques(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_joint_torques", data)
        pass

    def test_get_joint_temp(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_joint_temp", data)
        pass

    def test_get_joint_temperatures(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("get_joint_temperatures", data)
        pass

    def test_kinematics_forward(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("kinematics_forward", data)
        pass

    def test_kinematics_inverse(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("kinematics_inverse", data)
        pass

    def test_pose_times(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("pose_times", data)
        pass

    def test_pose_inverse(self):
        # 暂不支持

        # data = {}
        # self.http_service.action("pose_inverse", data)
        pass

    # 手爪控制

    def test_set_claw_force(self):
        data = {"type": "Force", "value": 50}
        r = self.http_service.action("set_claw_ao", data)
        print(r)
        pass

    def test_set_claw_claw(self):
        data = {"type": "Amplitude", "value": 50}
        r = self.http_service.action("set_claw_ao", data)
        print(r)
        pass

    def test_get_claw_force(self):
        data = {"type": "Force"}
        r = self.http_service.action("get_claw_ai", data)
        print(r)
        pass

    def test_get_claw_claw(self):
        data = {"type": "Amplitude"}
        r = self.http_service.action("get_claw_ai", data)
        pass

    # 机器人 I/O

    def test_set_do(self):
        data = {
            "pin": 1,
            "value": 1
        }
        self.http_service.action("set_do", data)
        pass

    def test_get_di(self):
        data = {
            "pin": 1
        }
        self.http_service.action("get_di", data)
        pass

    #
    # def test_wait_di(self):
    #     # pin, value, relation
    #
    #     # data = {}
    #     # self.http_service.action("wait_di", data)
    #     pass

    def test_set_flange_do(self):
        data = {
            "pin": 1,
            "value": 1
        }
        self.http_service.action("set_flange_do", data)
        pass

    def test_get_flange_di(self):
        data = {
            "pin": 1
        }
        self.http_service.action("get_flange_di", data)
        pass

    #
    # def test_wait_flange_di(self):
    #     # pin, value, relation
    #
    #     # data = {}
    #     # self.http_service.action("wait_flange_di", data)
    #     pass

    def test_set_aio(self):
        data = {
            "pin": 1,
            "value": 14.0
        }
        self.http_service.action("set_ao", data)
        pass

    def test_get_aio(self):
        data = {
            "pin": 1
        }
        self.http_service.action("get_ai", data)
        pass

    # def test_wait_aio(self):
    #     # pin, value, relation
    #
    #     # data = {}
    #     # self.http_service.action("wait_aio", data)
    #     pass
    #
    # def test_set_tcp_aio(self):
    #     # pin, value
    #
    #     # data = {}
    #     # self.http_service.action("set_tcp_aio", data)
    #     pass
    #
    # def test_get_tcp_aio(self):
    #     # pin
    #
    #     # data = {}
    #     # self.http_service.action("get_tcp_aio", data)
    #     pass
    #
    # def test_wait_tcp_aio(self):
    #     # pin, value, relation
    #
    #     # data = {}
    #     # self.http_service.action("wait_tcp_aio", data)
    #     pass

    # 外置设备 I/O

    def test_get_external_di(self):
        # deviceId, pinId

        # data = {}
        # self.http_service.action("get_external_di", data)
        pass

    def test_wait_external_di(self):
        # deviceId, pinId, value, relation

        # data = {}
        # self.http_service.action("wait_external_di", data)
        pass

    def test_set_external_do(self):
        # deviceId, pinId, value

        # data = {}
        # self.http_service.action("set_external_do", data)
        pass

    def test_get_external_do(self):
        # deviceId, pinId

        # data = {}
        # self.http_service.action("get_external_do", data)
        pass

    def test_get_external_ai(self):
        # deviceId, pinId

        # data = {}
        # self.http_service.action("get_external_ai", data)
        pass

    def test_wait_external_ai(self):
        # deviceId, pinId, value, relation

        # data = {}
        # self.http_service.action("wait_external_ai", data)
        pass

    def test_set_external_ao(self):
        # deviceId, pinId, value

        # data = {}
        # self.http_service.action("set_external_ao", data)
        pass

    def test_get_external_ao(self):
        # deviceId, pinId

        # data = {}
        # self.http_service.action("get_external_ao", data)
        pass

    def test_set_external_dos2(self):
        #

        # data = {}
        # self.http_service.action("set_external_dos2", data)
        pass

    def test_get_external_dos2(self):
        #

        # data = {}
        # self.http_service.action("get_external_dos2", data)
        pass

    def test_get_external_dis2(self):
        #

        # data = {}
        # self.http_service.action("get_external_dis2", data)
        pass

    def test_set_external_aos2(self):
        #

        # data = {}
        # self.http_service.action("set_external_aos2", data)
        pass

    def test_get_external_aos2(self):
        #

        # data = {}
        # self.http_service.action("get_external_aos2", data)
        pass

    def test_get_external_ais2(self):
        #

        # data = {}
        # self.http_service.action("get_external_ais2", data)
        pass

    def test_get_external_ios2(self):
        #

        # data = {}
        # self.http_service.action("get_external_ios2", data)
        pass

    # 附件控制

    def test_set_led(self):
        # mode, speed, color

        # data = {}
        # self.http_service.action("set_led", data)
        pass

    def test_set_voice(self):
        # voice, volume

        # data = {}
        # self.http_service.action("set_voice", data)
        pass

    def test_set_fan(self):
        # status

        # data = {}
        # self.http_service.action("set_fan", data)
        pass

    # 程序控制

    def test_print(self):
        # line

        # data = {}
        # self.http_service.action("print", data)
        pass

    def test_scene(self):
        # id

        # data = {}
        # self.http_service.action("scene", data)
        pass

    def test_alert(self):
        # msg

        # data = {}
        # self.http_service.action("alert", data)
        pass

    def test_confirm(self):
        # msg

        # data = {}
        # self.http_service.action("confirm", data)
        pass

    def test_input(self):
        # msg

        # data = {}
        # self.http_service.action("input", data)
        pass

    def test_option(self):
        # msg, options, cnt

        # data = {}
        # self.http_service.action("option", data)
        pass


if __name__ == '__main__':
    unittest.main(verbosity=1)
