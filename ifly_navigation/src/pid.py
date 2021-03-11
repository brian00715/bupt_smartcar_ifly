#!/usr/bin/env python
# coding=utf-8
import math
from matplotlib import pyplot as plt
import time
import random


class PID_t:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.now_value = 0
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.delta_value = 0
        self.expect_value = 0

    def get_output_delta(self, now_value, expect_value):
        """增量式PID计算方法"""
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_value = now_value
        self.now_err = expect_value - now_value
        self.delta_value = self.kp * (self.now_err - self.last_err) + self.ki * \
                           self.now_err + self.kd * (self.now_err - 2 * self.last_err
                                                     + self.last_last_err)
        # print (self.delta_value)
        return self.now_value+self.delta_value

    def get_output(self,now_value, expect_value):
        self.now_err = expect_value - now_value
        output = self.kp * self.now_err + 0 + self.kd * (self.now_err - self.last_err)
        self.last_err = self.now_err
        self.now_value = output
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
