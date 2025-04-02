#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from recharge.srv import Recharge, RechargeRequest, DisRecharge, DisRechargeRequest

def call_recharge_service():
    rospy.wait_for_service('/recharge')
    try:
        recharge_service = rospy.ServiceProxy('/recharge', Recharge)
        recharge_x = float(input("Enter recharge_x: "))
        recharge_y = float(input("Enter recharge_y: "))
        recharge_yaw = float(input("Enter recharge_yaw: "))
        req = RechargeRequest(recharge_x=recharge_x, recharge_y=recharge_y, recharge_yaw=recharge_yaw)
        resp = recharge_service(req)
        print("Result: ", resp.result)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def call_disrecharge_service():
    rospy.wait_for_service('/dis_recharge')
    try:
        disrecharge_service = rospy.ServiceProxy('/dis_recharge', DisRecharge)
        empty_string_msg = String(data="")
        req = DisRechargeRequest(dis_recharge=empty_string_msg)
        resp = disrecharge_service(req)
        print("Result: ", resp.result)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('service_caller')

    try:
        while not rospy.is_shutdown():
            service_choice = input("Enter 1 to call Recharge service, 2 to call DisRecharge service: ")

            if service_choice == '1':
                call_recharge_service()
            elif service_choice == '2':
                call_disrecharge_service()
            else:
                print("Invalid choice. Please enter 1 or 2.")
    except KeyboardInterrupt:
        print("Shutting down the service caller.")

