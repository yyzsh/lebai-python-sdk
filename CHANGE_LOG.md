# 更改日志

## 0.6.0

- 添加获取任务、执行lua代码方法
- 修复add_signal、rerun_task 参数错误
- 添加单元测试
- 修复Robot run一直不返回值 bug
- 去除一些无效的 _sync调用
- 添加方法注释与类型注释
- 自动生成API文档
- 添加类型：RobotIOData、IOItem、RobotData、RobotPoseData、PVAT
    - robot类 move_pvats、move_pvts、move_pts 函数参数由字典改为使用 PVAT 类
    - robot类 get_robot_io_data 返回类型由字典改为使用 RobotIOData类型
    - robot类 get_robot_data 返回类型由字典改为使用 RobotData类型
    - robot类 get_robot_poses 返回类型由字典改为使用 RobotPoseData类型

## 0.6.1

- 修复模拟信号类型错误


## 0.6.2

- 修复 get_tasks 返回结果

## 0.6.3

- 删除list[] 写法的注释，兼容低版本python

## 0.6.4

- 修复`CartesianPose __getattr__`导致的递归调用，内存溢出 bug

## 0.6.5

- fix`LebaiScene` `result` type error
- 更改`tuple`到`list`