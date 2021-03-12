#!/usr/bin/env python
# coding=utf-8
import math
from matplotlib import pyplot as plt
import time
import random
import rospy


class PID_t:
    def __init__(self, kp, ki, kd, int_duty=0.01, int_max=100, output_max=10,sub_ctrl = False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.now_value = 0
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.delta_value = 0
        self.int_count = 0  # 积分量
        self.expect_value = 0
        self.int_duty = int_duty  # 积分周期/s
        self.curr_time = rospy.get_time()
        self.last_time = 0
        self.int_max = int_max  # 积分限额
        self.output_max = output_max
        self.sub_ctrl = sub_ctrl # 是否开启分段pid

    def get_output_delta(self, now_value, expect_value):
        """增量式PID计算方法"""
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_value = now_value
        self.now_err = expect_value - now_value
        self.delta_value = self.kp * (self.now_err - self.last_err) + self.ki * \
                           self.now_err + self.kd * (self.now_err - 2 * self.last_err
                                                     + self.last_last_err)
        return self.now_value + self.delta_value

    def get_output(self, now_value, expect_value):
        self.now_err = expect_value - now_value
        self.int_count += self.now_err
        if self.int_count >= self.int_max:
            self.int_count = self.int_max
        if self.int_count <= - self.int_max:
            self.int_count = -self.int_max

        output = 0
        if self.sub_ctrl == False:
            output = self.kp * self.now_err + self.ki * self.int_count + self.kd * (self.now_err - self.last_err)
        else:
            if abs(self.now_err) > 0.2:
                output = self.kp * self.now_err + self.ki * self.int_count + self.kd * (self.now_err - self.last_err)
            else:
                output = 0.01 * self.now_err + 0.15 * self.int_count +2 * (self.now_err - self.last_err)

        if output > self.output_max:
            output = self.output_max
        if output < -self.output_max:
            output = -self.output_max
        self.last_err = self.now_err
        return output


if __name__ == '__main__':
    pid = PID_t(0.1, 0, 0.1)
    x = []
    y = []
    for i in range(50):
        y_sample = pid.get_output(pid.now_value, 5000)
        print (y_sample)
        y.append(y_sample)
    plt.plot(y)
    plt.show()
