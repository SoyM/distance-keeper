#!/usr/bin/env python
# -*-coding:utf-8 -*-

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import serial
import time
import subprocess
from multiprocessing import Process
import os
import sys
import signal
from timeout_decorator.timeout_decorator import timeout, TimeoutError
import tf
from nav_msgs.msg import Odometry
import math


def sigint_handler(signum, frame):
    global is_sigint_up
    is_sigint_up = True
    print ('serial_to_ros catched interrupt signal!')
    exit(0)


signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGHUP, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)
err_count = 0
hello_str = '0'
online_flag = False
dev_name = '/dev/ttyUSB0'
ser = ''
adc_left = 0
adc_right = 0
tof_a = 0

# 2019.02.19
yaw = 0
linear_x = 0
linear_y = 0


@timeout(seconds=5, use_signals=True)
def read_serial():
    global err_count
    global hello_str, adc_left, adc_right, tof_a
    global online_flag
    global dev_name
    global ser

    try:
        ser = serial.Serial(dev_name, 115200)
    except serial.serialutil.SerialException:
        print("[Error] : device disconnected or multiple access on port?")
        exit(1)

    check_count = 0
    check_count_max = 200  # loop最大阈值:和两段有效字符之间的 无效字符数 有关 (不能过小,过大只会影响速率)
    while check_count < check_count_max:

        if ser.read() is "d":
            if ser.read() is ":":
                            break
        check_count = check_count + 1
    print("check_count: {check_count}").format(check_count=check_count)

# 多次读串口字符 未匹配,发送上一次读数值.累计达20次发送上一次读数值,退出.
    if check_count != check_count_max:
        hello_str = ''
        adc_left = ''
        adc_right = ''
        tof_a = ''
        for n in range(0, 5):
            serial_read_data = ser.read()
            if serial_read_data == ' ':
                break
            else:
                hello_str = str(hello_str)+str(serial_read_data)
        # print(hello_str)

        ser.read()  # 丢弃一个空格
        for n in range(0, 4):
            serial_read_data = ser.read()
            if serial_read_data == ' ':
                break
            else:
                adc_left = str(adc_left)+str(serial_read_data)

        for n in range(0, 4):
            serial_read_data = ser.read()
            if serial_read_data == ' ':
                break
            else:
                adc_right = str(adc_right)+str(serial_read_data)

        for n in range(0, 5):
            serial_read_data = ser.read()
            if serial_read_data == ' ':
                break
            else:
                tof_a = str(tof_a)+str(serial_read_data)

        print("adc_left:{adc_left}, adc_right:{adc_right}, tof: {tof_a}".format(
            adc_left=adc_left, adc_right=adc_right, tof_a=tof_a))

        err_count = 0

    else:
        err_count = err_count + 1

    print("err_count : {err_count}".format(err_count=err_count))

    if err_count > 20:
        print("err count 20 times,restart..")
        err_count = 0
        exit(0)
    elif err_count > 10:
        online_flag = False
    else:
        online_flag = True


def odom_callback(msg):
    global yaw, linear_x, linear_y
    linear_x = msg.pose.pose.position.x
    linear_y = msg.pose.pose.position.y
    (r, p, yaw) = tf.transformations.euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    # print("r:{r}, p:{p}, y:{y}".format(r=r, p=p, y=yaw))


def light_status():
    global err_count
    global online_flag
    global dev_name
    global hello_str, adc_left, adc_right, tof_a
    global yaw, linear_x, linear_y
    global ser
    err_count = 0
    online_flag = False
    # hello_str = ''

    pub = rospy.Publisher('irAdc_diff', Int32, queue_size=10)
    pub_cmdvel = rospy.Publisher(
        '/cmd_vel_mux/input/navi', Twist, queue_size=10)
    sub_odom = rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.init_node('ir_double', anonymous=False)
    rate = rospy.Rate(1000)

    #
    # 找设备路径
    #
    line_mix = []
    while len(line_mix) == 0:

        p = subprocess.Popen(
            "find /sys/bus/usb/devices/usb*/ -name dev | grep ttyUSB", shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while True:
            line = p.stdout.readline()
            if not line:
                break
            line = line.strip()
            # print('Subprogram output: [{}]'.format(line))
            line_mix.append(line.decode())
            # print("subprocess returncode {returncode}".format(
            #     returncode=p.returncode))
        print("found Num of USB : {len}".format(len=len(line_mix)))
        time.sleep(1)

    check_model_id = False
    check_vendor_id = False

    for single_path in line_mix:
        # print(single_path[:-4] + "\n---------")
        dev_property = subprocess.Popen("udevadm info -q property --export -p " +
                                        single_path[:-4], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        while True:
            line = dev_property.stdout.readline()
            if not line:
                break
            line = line.strip()
            # if line:
            # print('Subprogram output: {}'.format(line))
            temp_line = line.decode().split("=")
            # print(temp_line[0])

            if temp_line[0] == 'ID_MODEL_ID' and temp_line[1][1:-1] == "7523":
                check_model_id = True
            if temp_line[0] == 'ID_VENDOR_ID' and temp_line[1][1:-1] == "1a86":
                check_vendor_id = True
            if temp_line[0] == 'DEVNAME':
                dev_name = temp_line[1][1:-1]

        if check_model_id == True and check_vendor_id == True:
            break
        else:
            dev_name = '/dev/ttyUSB0'
        # property_arr.append(temp_line_dict)

    print(dev_name)

    move_cmd = Twist()
    time_count = 0
    too_far = False
    too_closer = False
    virgin = True
    can_stop = False

    min_tof_a = 2000

###########################################
# loop start
###########################################

    while not rospy.is_shutdown():
        # print("rosdistro: {rosout}".format(
        #     rosout=rospy.get_param("rosdistro").strip()))
        read_serial()

        if virgin:
            virgin = False
            virgin_count = 0
            while virgin_count<30:
                read_serial()
                virgin_count=virgin_count+1
        
            print("[\033[1;32;40mstart\033[0m]")

            if int(adc_right) < 70:

                yaw_start = yaw
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0.4
                while True:
                    read_serial()

                    # bugs
                    if yaw_start > 0 and yaw < 0:
                        diff_yaw = yaw + (math.pi*2) - yaw_start
                    else:
                        diff_yaw = yaw - yaw_start

                    # 当 垂直时 距离 最小
                    if tof_a != '':
                        if int(tof_a)<min_tof_a:
                            min_tof_a = int(tof_a)
                        if  diff_yaw > math.pi*1/3:
                            if int(tof_a) > min_tof_a + 4:
                                read_serial()
                                if int(tof_a) > min_tof_a + 4:
                                    print("\033[0;32mfirst rotate end by [tof] \033[0m")
                                    break

                    if diff_yaw > math.pi/2:
                        print("\033[0;32mfirst rotate end [odom] \033[0m")
                        break
                    else:
                        pub_cmdvel.publish(move_cmd)
                        rate.sleep()

                # go straight
                move_cmd.linear.x = 0.05
                move_cmd.angular.z = 0
                # x_start = linear_x
                # y_start = linear_y
                # while True:
                #     diff_distance = math.sqrt(
                #         math.pow(linear_x-x_start, 2) + math.pow(linear_y-y_start, 2))
                #     print("diff_distance: {diff_distance}".format(
                #         diff_distance=diff_distance))
                #     if diff_distance > 0.26:
                #         break
                #     else:
                #         pub_cmdvel.publish(move_cmd)
                #         rate.sleep()

                # tof
                while True:
                    read_serial()
                    if int(tof_a)<110:
                        print("\033[0;32mgo straight end [tof_a<110] \033[0m")
                        break
                    else:
                        pub_cmdvel.publish(move_cmd)
                        rate.sleep()

                # recovery
                yaw_start = yaw
                move_cmd.linear.x = 0
                move_cmd.angular.z = -0.4
                while True:
                    # bugs
                    if yaw_start<0 and yaw>0:
                        yaw_diff = yaw - math.pi*2 - yaw_start
                    else:
                        yaw_diff = yaw-yaw_start
                    if yaw_diff < -math.pi/2:
                        print("\033[0;32msencond rotate end [odom]\033[0m")
                        break
                    else:
                        pub_cmdvel.publish(move_cmd)
                        rate.sleep()
            ser.write("F")
            start_time = time.time()
###########################################
# main loop 
###########################################
        if online_flag:
            rospy.loginfo(hello_str)
            pub.publish(int(hello_str))
            print("[\033[1;32;40mpublished\033[0m]")

# common scenario
        move_cmd.linear.x = 0.05
        move_cmd.angular.z = 0

# too far or too closer
        if int(adc_right) > 160:
            if too_closer == True:
                time_count = 0

            time_count = time_count + 1
            too_far = True
            too_closer = False

        if int(adc_right) < 140:
            if too_far == True:
                time_count = 0

            time_count = time_count + 1
            too_closer = True
            too_far = False

        if time_count > 1:
            time_count = 0
            foo_count = 0
            while foo_count < 35:
                if too_closer:
                    move_cmd.angular.z = 0.25
                else:
                    move_cmd.angular.z = -0.25
                pub_cmdvel.publish(move_cmd)
                foo_count = foo_count+1
                rate.sleep()

            too_closer = False
            too_far = False


# alaways adjust
        if int(adc_right) < 160 and int(adc_right) > 140:
            time_count = 0
            too_far = False
            too_closer = False

            aSpeed_min = 0.2
            aSpeed_max = 0.5
            if int(hello_str) < -18:
                if int(hello_str) < -70:
                    move_cmd.angular.z = -aSpeed_max
                else:
                    move_cmd.angular.z = -aSpeed_max * (-int(hello_str)/70)
                move_cmd.angular.z = move_cmd.angular.z if move_cmd.angular.z > - \
                    aSpeed_min else -aSpeed_min

            elif int(hello_str) > 18:
                if int(hello_str) > 70:
                    move_cmd.angular.z = aSpeed_max
                else:
                    move_cmd.angular.z = aSpeed_max * (int(hello_str)/70)
                move_cmd.angular.z = move_cmd.angular.z if move_cmd.angular.z < aSpeed_min else aSpeed_min
        else:
            move_cmd.angular.z = 0
        if can_stop == False:
            if time.time()-start_time > 20:
                can_stop = True
        if can_stop:
            if int(adc_right) < 80:
                print("kkkkkkkkkkkkkkkkkk")
                sys.exit()
        pub_cmdvel.publish(move_cmd)

        rate.sleep()


if __name__ == '__main__':

    # while True:
    print('Parent process %s.' % os.getpid())
    p = Process(target=light_status)
    print('Child process will start.')
    p.start()
    p.join()
    print('Child process end.')

