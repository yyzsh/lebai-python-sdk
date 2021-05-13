#!/usr/bin/python
# -*- coding: utf-8 -*-
import json
import unittest

from lebai.lebai_http_service import LebaiHttpService


class Test(unittest.TestCase):
    """Test test.py"""
    ip = "192.168.3.227"

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        self.http_service = LebaiHttpService(self.ip)

    def tearDown(self):
        pass


    def get_task_id(self):
        code = "print('test')"
        return self.http_service.execute_lua_code('任务名称', 1, 1, code)['id']

    def test_get_tasks(self):
        self.http_service.get_tasks(0, 10)


    def test_execute_lua_code(self):
        code = "print('test')"
        self.http_service.execute_lua_code('任务名称', 1, 1, code)

    def test_get_task(self):
        self.http_service.get_task(self.get_task_id())

    def test_run_task(self):
        self.http_service.run_task(self.get_task_id(), 1, 1)
        pass

    def test_run_scene(self):
        self.http_service.run_task(10001, 1, 1)
        pass

    # 状态控制

    def test_start_sys(self):
        # data = json.dumps({})
        # self.http_service.action("start_sys", data)
        pass

    def test_stop_sys(self):
        # data = json.dumps({})
        # self.http_service.action("stop_sys", data)
        pass

    def test_powerdown(self):
        # data = json.dumps({})
        # self.http_service.action("powerdown", data)
        pass

    def test_stop(self):
        # data = json.dumps({})
        # self.http_service.action("stop", data)
        pass

    def test_estop(self):
        # data = json.dumps({})
        # self.http_service.action("estop", data)
        pass

    def test_teach_mode(self):
        # data = json.dumps({})
        # self.http_service.action("teach_mode", data)
        pass

    def test_end_teach_mode(self):
        # data = json.dumps({})
        # self.http_service.action("end_teach_mode", data)
        pass

    def test_pause_task(self):
        # data = json.dumps({})
        # self.http_service.action("pause_task", data)
        pass

    def test_resume_task(self):
        # data = json.dumps({})
        # self.http_service.action("resume_task", data)
        pass

    def test_wait(self):
        # data = json.dumps({})
        # self.http_service.action("wait", data)
        pass

    def test_wait_until(self):
        # data = json.dumps({})
        # self.http_service.action("wait_until", data)
        pass

    def test_stop_task(self):
        # data = json.dumps({})
        # self.http_service.action("stop_task", data)
        pass

    def test_get_task_status(self):
        # data = json.dumps({})
        # self.http_service.action("get_task_status", data)
        pass

    # 参数配置

    def test_set_tcp(self):
        # data = json.dumps({})
        # self.http_service.action("set_tcp", data)
        pass

    def test_get_tcp(self):
        # data = json.dumps({})
        # self.http_service.action("get_tcp", data)
        pass

    def test_set_velocity_factor(self):
        # data = json.dumps({})
        # self.http_service.action("set_velocity_factor", data)
        pass

    def test_get_velocity_factor(self):
        # data = json.dumps({})
        # self.http_service.action("get_velocity_factor", data)
        pass

    def test_set_payload(self):
        # data = json.dumps({})
        # self.http_service.action("set_payload", data)
        pass

    def test_get_payload(self):
        # data = json.dumps({})
        # self.http_service.action("get_payload", data)
        pass

    def test_set_payload_mass(self):
        # data = json.dumps({})
        # self.http_service.action("set_payload_mass", data)
        pass

    def test_get_payload_mass(self):
        # data = json.dumps({})
        # self.http_service.action("get_payload_mass", data)
        pass

    def test_set_payload_cog(self):
        # data = json.dumps({})
        # self.http_service.action("set_payload_cog", data)
        pass

    def test_get_payload_cog(self):
        # data = json.dumps({})
        # self.http_service.action("get_payload_cog", data)
        pass

    def test_set_gravity(self):
        # data = json.dumps({})
        # self.http_service.action("set_gravity", data)
        pass

    def test_get_gravity(self):
        # data = json.dumps({})
        # self.http_service.action("get_gravity", data)
        pass

    # 移动命令

    def test_movej(self):
        # data = json.dumps({})
        # self.http_service.action("movej", data)
        pass

    def test_movel(self):
        # data = json.dumps({})
        # self.http_service.action("movel", data)
        pass

    def test_movec(self):
        # data = json.dumps({})
        # self.http_service.action("movec", data)
        pass

    def test_stop_move(self):
        # data = json.dumps({})
        # self.http_service.action("stop_move", data)
        pass

    def test_stop_move(self):
        # data = json.dumps({})
        # self.http_service.action("stop_move", data)
        pass

    def test_movej_until(self):
        # data = json.dumps({})
        # self.http_service.action("movej_until", data)
        pass

    def test_movel_until(self):
        # data = json.dumps({})
        # self.http_service.action("movel_until", data)
        pass

    def test_movec_until(self):
        # data = json.dumps({})
        # self.http_service.action("movec_until", data)
        pass

    def test_movej_until_rt(self):
        # data = json.dumps({})
        # self.http_service.action("movej_until_rt", data)
        pass

    def test_movel_until_rt(self):
        # data = json.dumps({})
        # self.http_service.action("movel_until_rt", data)
        pass

    def test_movec_until_rt(self):
        # data = json.dumps({})
        # self.http_service.action("movec_until_rt", data)
        pass

    # 状态数据

    def test_robot_data(self):
        #

        # data = json.dumps({})
        # self.http_service.action("robot_data", data)
        pass

    def test_get_robot_mode(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_robot_mode", data)
        pass

    def test_get_actual_joint_positions(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_actual_joint_positions", data)
        pass

    def test_get_actual_joint_speeds(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_actual_joint_speeds", data)
        pass

    def test_get_target_joint_positions(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_target_joint_positions", data)
        pass

    def test_get_target_joint_speeds(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_target_joint_speeds", data)
        pass

    def test_get_actual_tcp_pose(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_actual_tcp_pose", data)
        pass

    def test_get_target_tcp_pose(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_target_tcp_pose", data)
        pass

    def test_get_joint_torques(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_joint_torques", data)
        pass

    def test_get_joint_temp(self):
        # joint

        # data = json.dumps({})
        # self.http_service.action("get_joint_temp", data)
        pass

    def test_get_joint_temperatures(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_joint_temperatures", data)
        pass

    def test_kinematics_forward(self):
        # joints

        # data = json.dumps({})
        # self.http_service.action("kinematics_forward", data)
        pass

    def test_kinematics_inverse(self):
        # vector

        # data = json.dumps({})
        # self.http_service.action("kinematics_inverse", data)
        pass

    def test_pose_times(self):
        # a, b

        # data = json.dumps({})
        # self.http_service.action("pose_times", data)
        pass

    def test_pose_inverse(self):
        # a

        # data = json.dumps({})
        # self.http_service.action("pose_inverse", data)
        pass

    # 手爪控制
    def test_set_claw(self):
        # force, amplitude

        # data = json.dumps({})
        # self.http_service.action("set_claw", data)
        pass

    def test_set_claw_aio(self):
        # type, value

        # data = json.dumps({})
        # self.http_service.action("set_claw_aio", data)
        pass

    def test_get_claw_aio(self):
        # type

        # data = json.dumps({})
        # self.http_service.action("get_claw_aio", data)
        pass

    def test_wait_claw_aio(self):
        # type, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_claw_aio", data)
        pass

    # 机器人 I/O

    def test_set_do(self):
        # pin, value

        # data = json.dumps({})
        # self.http_service.action("set_do", data)
        pass

    def test_get_di(self):
        # pin

        # data = json.dumps({})
        # self.http_service.action("get_di", data)
        pass

    def test_wait_di(self):
        # pin, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_di", data)
        pass

    def test_set_flange_dio(self):
        # pin, value

        # data = json.dumps({})
        # self.http_service.action("set_flange_dio", data)
        pass

    def test_get_flange_di(self):
        # pin

        # data = json.dumps({})
        # self.http_service.action("get_flange_di", data)
        pass

    def test_wait_flange_di(self):
        # pin, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_flange_di", data)
        pass

    def test_set_aio(self):
        # pin, value

        # data = json.dumps({})
        # self.http_service.action("set_aio", data)
        pass

    def test_get_aio(self):
        # pin

        # data = json.dumps({})
        # self.http_service.action("get_aio", data)
        pass

    def test_wait_aio(self):
        # pin, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_aio", data)
        pass

    def test_set_tcp_aio(self):
        # pin, value

        # data = json.dumps({})
        # self.http_service.action("set_tcp_aio", data)
        pass

    def test_get_tcp_aio(self):
        # pin

        # data = json.dumps({})
        # self.http_service.action("get_tcp_aio", data)
        pass

    def test_wait_tcp_aio(self):
        # pin, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_tcp_aio", data)
        pass

    # 外置设备 I/O

    def test_get_external_di(self):
        # deviceId, pinId

        # data = json.dumps({})
        # self.http_service.action("get_external_di", data)
        pass

    def test_wait_external_di(self):
        # deviceId, pinId, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_external_di", data)
        pass

    def test_set_external_do(self):
        # deviceId, pinId, value

        # data = json.dumps({})
        # self.http_service.action("set_external_do", data)
        pass

    def test_get_external_do(self):
        # deviceId, pinId

        # data = json.dumps({})
        # self.http_service.action("get_external_do", data)
        pass

    def test_get_external_ai(self):
        # deviceId, pinId

        # data = json.dumps({})
        # self.http_service.action("get_external_ai", data)
        pass

    def test_wait_external_ai(self):
        # deviceId, pinId, value, relation

        # data = json.dumps({})
        # self.http_service.action("wait_external_ai", data)
        pass

    def test_set_external_ao(self):
        # deviceId, pinId, value

        # data = json.dumps({})
        # self.http_service.action("set_external_ao", data)
        pass

    def test_get_external_ao(self):
        # deviceId, pinId

        # data = json.dumps({})
        # self.http_service.action("get_external_ao", data)
        pass

    def test_set_external_dos2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("set_external_dos2", data)
        pass

    def test_get_external_dos2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_external_dos2", data)
        pass

    def test_get_external_dis2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_external_dis2", data)
        pass

    def test_set_external_aos2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("set_external_aos2", data)
        pass

    def test_get_external_aos2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_external_aos2", data)
        pass

    def test_get_external_ais2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_external_ais2", data)
        pass

    def test_get_external_ios2(self):
        #

        # data = json.dumps({})
        # self.http_service.action("get_external_ios2", data)
        pass

    # 附件控制

    def test_set_led(self):
        # mode, speed, color

        # data = json.dumps({})
        # self.http_service.action("set_led", data)
        pass

    def test_set_voice(self):
        # voice, volume

        # data = json.dumps({})
        # self.http_service.action("set_voice", data)
        pass

    def test_set_fan(self):
        # status

        # data = json.dumps({})
        # self.http_service.action("set_fan", data)
        pass

    # 程序控制

    def test_print(self):
        # line

        # data = json.dumps({})
        # self.http_service.action("print", data)
        pass

    def test_scene(self):
        # id

        # data = json.dumps({})
        # self.http_service.action("scene", data)
        pass

    def test_alert(self):
        # msg

        # data = json.dumps({})
        # self.http_service.action("alert", data)
        pass

    def test_confirm(self):
        # msg

        # data = json.dumps({})
        # self.http_service.action("confirm", data)
        pass

    def test_input(self):
        # msg

        # data = json.dumps({})
        # self.http_service.action("input", data)
        pass

    def test_option(self):
        # msg, options, cnt

        # data = json.dumps({})
        # self.http_service.action("option", data)
        pass

if __name__ == '__main__':
    unittest.main(verbosity=1)
