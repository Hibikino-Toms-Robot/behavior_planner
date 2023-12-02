#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import smach
from smach import State, StateMachine

from smach_ros import ServiceState
from std_msgs.msg import Bool
from std_msgs.msg import String
import time
import smach_ros
import numpy as np

import sys
sys.path.append("/home/hibikinotoms/hibikino_toms_ws/src/behavior_planner/behavior_planner")
from relay_modul import Main_Circuit_Control

import time
from toms_msg.srv import CartService, VisionService, ArmService ,EndEffectorService
from toms_msg.msg import TomatoPos,TomatoData
"""
@autor keisuke yoshida
--------------------------
状態遷移(行動決定)プログラム

トマトロボット2号機は4つの状態を持ち,以下のような動作を行う
[1]Setup    : リレー接続,アーム原点位置移動
[2]Move     : カート移動,トマト
[3]Analyze  : トマト3次元座標位置取得
[4]Harvest  : 収穫動作,収穫判定

"""

rclpy.init()
node = Node('behavior_planner')

class Setup(State):
    """
    初期動作確認
    ・リレー接続
    ・各種アームの原点回帰
    """
    def __init__(self):
        State.__init__(self, outcomes=['setup_done'])

        self.cli = node.create_client(ArmService, "arm_service") 
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('arm_service not available')
        self.req = ArmService.Request() 
    
    def arm_send_request(self):
        self.req.task = "init_arm"
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(node, self.future)
        return self.future.result()

    def main_circuit_setup(self):
        msc = Main_Circuit_Control()
        mode = True
        msc.send_relay_command(mode)

    def execute(self, userdata):
        node.get_logger().info("Relay connection")
        self.main_circuit_setup()
        node.get_logger().info("Arm Init complete")
        self.arm_send_request()
        node.get_logger().info("Initial setup complete")
        time.sleep(5) #動作開始までの時間
        return 'setup_done'
    
class Move(State):
    def __init__(self):
        State.__init__(self, outcomes=['analyze_mode','continue_moving','fin_moving'])
        self.cart_direction = 1 #１が前進,-1が後退
        
        self.vision_cli = node.create_client(VisionService,"vision_service") 
        while not self.vision_cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('Waiting for vision_node service')
        self.vision_req = VisionService.Request() 

        self.cart_cli = node.create_client(CartService,"cart_service") 
        while not self.cart_cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('Waiting for cart_node service')
        self.cart_req = CartService.Request() 

    def vision_send_request(self):
        self.vision_req.task = "detect_check"
        self.vision_future = self.vision_cli.call_async(self.vision_req)
        rclpy.spin_until_future_complete(node, self.vision_future)
        return self.vision_future.result()
    
    def cart_send_request(self):
        self.cart_future = self.cart_cli.call_async(self.cart_req)
        rclpy.spin_until_future_complete(node, self.cart_future)
        return self.cart_future.result()
       
    def execute(self, userdata):
        if self.cart_direction == 1 :
            self.cart_req.command = "forword"
        elif self.cart_direction == -1 :
            self.cart_req.command = "back"
        else :
            self.cart_req.command = "stop"    
        response_cart = self.cart_send_request()
        try :
            node.get_logger().info(f"status: {response_cart.status.data}")
            node.get_logger().info(f"distanse: {response_cart.distance.data}")
        except :
            pass

        #　接触判定→動作逆転
        if response_cart.status.data == 1 or response_cart.status.data == 2 :
                self.cart_direction = self.cart_direction*-1
        elif self.cart_direction == -1 and response_cart.status.data == 3 :
            time.sleep(3) #物体にあたりそうな時は3秒待つ
            response_cart = self.cart_send_request()
            if response_cart.status.data != 0 :
                self.cart_direction = self.cart_direction*-1

        vision_response = self.vision_send_request()
        if vision_response is not None:
            if vision_response.detect_check:
                state='analyze_mode'
            else:
                state='continue_moving'
        else:
            print("e")
            state='continue_moving'
        node.get_logger().info(f"next : {state}")
        time.sleep(2)
        return state

class Analyze(State):
    def __init__(self):
        State.__init__(self, outcomes=['task_comp', 'continue_moving'],input_keys=['target_in'],output_keys=['target_out'])
        #service通信
        self.cli = node.create_client(VisionService,"vision_service") 
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('vision_service not available')
        self.vision_req = VisionService.Request() 

    def vision_send_request(self):
        self.vision_req.task = "req_tomato_pose"
        self.future = self.cli.call_async(self.vision_req)
        rclpy.spin_until_future_complete(node, self.future)
        return self.future.result()
      
    def execute(self, userdata):
        # トマト座標取得
        response = self.vision_send_request()     
        userdata.target_out = response.target_pos
        if response.target_pos.tomato_data :
            state = 'task_comp'
        else :
            state = 'continue_moving'
        node.get_logger().info(f"next : {state}")
        return state 

class Harvest(State):
    def __init__(self):
        State.__init__(self, outcomes=['continue_moving','continue_harvest'],input_keys=['target_in'],output_keys=['target_out'])
        self.harvest_counter=0
        self.cnt=0

        self.arm_cli = node.create_client(ArmService, "arm_service") 
        while not self.arm_cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('arm_service not available')
        self.arm_req = ArmService.Request() 

        self.end_effector_cli = node.create_client(EndEffectorService,"end_effector_service") 
        while not self.end_effector_cli.wait_for_service(timeout_sec=1.0): 
            node.get_logger().info('end_effector_service not available')
        self.end_effector_req = EndEffectorService.Request() 
    
    def arm_send_request(self):
        self.arm_future = self.arm_cli.call_async(self.arm_req)
        rclpy.spin_until_future_complete(node, self.arm_future)
        return self.arm_future.result()
    
    def end_effector_send_request(self):
        self.end_effector_req.task_start = True
        self.end_effector_future = self.end_effector_cli.call_async(self.end_effector_req)
        rclpy.spin_until_future_complete(node, self.end_effector_future)
        return self.end_effector_future.result()
        
    def execute(self,userdata):
        targets = userdata.target_in.tomato_data
        for target in targets :
            # node.get_logger().info(f"目標x : {target.x}")
            # node.get_logger().info(f"目標y : {target.y}")
            # node.get_logger().info(f"目標z : {target.z}")
            # node.get_logger().info(f"目標z : {target.approach_direction}")
            self.arm_req.target = target
            response = self.arm_send_request()
            node.get_logger().info(f"移動完了")
            response = self.end_effector_send_request()

        state = 'continue_moving'
        node.get_logger().info(f"next : {state}")
        return 'continue_moving'
        
def main():
    sm = smach.StateMachine(outcomes=['END']) 
    sm.userdata.target_pose = 0
    with sm :
        smach.StateMachine.add('Setup', Setup(), 
                        transitions={'setup_done': 'MOVE'}
                                    )
        smach.StateMachine.add('MOVE', Move(), 
                        transitions={'analyze_mode': 'ANALYZE',
                                    'continue_moving': 'MOVE',
                                    'fin_moving':'END'}
                                    )
        smach.StateMachine.add('ANALYZE', Analyze(), 
                        transitions={'task_comp': 'HARVEST',
                                    'continue_moving': 'MOVE'},
                                    remapping={'target_in':'target_pose', 
                                   'target_out':'target_pose'}
                                    )
        smach.StateMachine.add('HARVEST', Harvest(), 
                         transitions={'continue_moving': 'MOVE',
                                    'continue_harvest': 'HARVEST'},
                                    remapping={'target_in':'target_pose', 
                                    'target_out':'target_pose'}
                                    )
    
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rclpy.spin(node) 
    sis.stop()

if __name__ == '__main__':
    main()