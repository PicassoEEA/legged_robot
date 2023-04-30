#!/usr/bin/env python3
#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import threading
import sqlite3 as sql
import numpy as np
import sys


from ServoCmd import setServoPulse

from PuppyInstantiate import PuppyInstantiate as puppy

# from HiwonderPuppy import HiwonderPuppy, PWMServoParams
# puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(), dof='8')

HomePath = '/home/pi'
# print('HomePath',HomePath)
runningAction = False
stopRunning = False
online_action_num = None
online_action_times = -1
update_ok = False
action_group_finish = True


def runActionGroup(num, wait = False):
    global runningAction
    threading.Thread(target=runAction, args=(num, )).start()
    if wait == False:return
    
    t = time.time() # 等待动作组做结束
    time.sleep(0.02)
    while time.time() - t < 30:#超时强制跳出
        time.sleep(0.001)
        if runningAction == False:
            break
        

def stopActionGroup():
    global stopRunning, online_action_num, online_action_times, update_ok
    update_ok = False
    stopRunning = True
    online_action_num = None
    online_action_times = -1
    time.sleep(0.1)

def stop_servo():
    for i in range(16):
        pass
        # stopBusServo(i+1) 

def action_finish():
    global action_group_finish
    return action_group_finish  

def runAction(actNum):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param times:  运行次数
    :return:
    '''
    global runningAction
    global stopRunning
    global online_action_times
    if actNum is None:
        return
    actNum = HomePath + "/PuppyPi_PC_Software/ActionGroups/" + actNum
    stopRunning = False
    if os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")

            puppy.servo_force_run()
            time.sleep(0.01)
            while True:
                act = cu.fetchone()
                if stopRunning is True:
                    stopRunning = False                   
                    break
                if act is not None:
                    if type(act[2]) is int:
                        for i in range(0, len(act)-2, 1):
                            setServoPulse(i+1, act[2 + i], act[1])
                                
                    elif type(act[2]) is float:
                        rotated_foot_locations = np.zeros(12)
                        for i in range(0, len(act)-2):
                            value = act[i+2]
                            rotated_foot_locations[i] = float(value)
                        rotated_foot_locations = rotated_foot_locations.reshape(4,3)
                        rotated_foot_locations = rotated_foot_locations.T
                        rotated_foot_locations = rotated_foot_locations/100
                        joint_angles = puppy.fourLegsRelativeCoordControl(rotated_foot_locations)
                        
                        puppy.sendServoAngle(joint_angles, act[1])#, force_execute = True
                        # joint_angles = puppy.four_legs_inverse_kinematics_relative_coord(rotated_foot_locations, puppy.config)
                        # puppy.send_servo_commands(PWMServoParams(), joint_angles, act[1])
                        

                    time.sleep(float(act[1])/1000.0)
                else:   # 运行完才退出
                    break

            runningAction = False
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件")

def online_thread_run_acting():
    global online_action_times, online_action_num, update_ok, action_group_finish
    while True:
        if update_ok:
            if online_action_times == 0:
                # 无限次运行
                if action_group_finish:
                    action_group_finish = False
                runAction(online_action_num)                
            elif online_action_times > 0:
                # 有次数运行
                if action_group_finish:
                    action_group_finish = False
                runAction(online_action_num)
                online_action_times -= 1    # 运行完成后，进入空载                
                if online_action_times == 0:
                    online_action_times = -1
            else:
                # 空载
                if not action_group_finish:
                    action_group_finish = True
                time.sleep(0.001)
        else:
            if not action_group_finish:
                action_group_finish = True
            time.sleep(0.001)
            
def start_action_thread():
    th1 = threading.Thread(target=online_thread_run_acting)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是True
    th1.start()
    
def change_action_value(actNum, actTimes):
    global online_action_times, online_action_num, update_ok, stopRunning, action_group_finish
    
    if action_group_finish:
        online_action_times = actTimes
        online_action_num = actNum
        stopRunning = False
        update_ok = True

from HandTrackerRenderer import HandTrackerRenderer
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-e', '--edge', action="store_true",
                    help="Use Edge mode (postprocessing runs on the device)")
parser_tracker = parser.add_argument_group("Tracker arguments")
parser_tracker.add_argument('-i', '--input', type=str, 
                    help="Path to video or image file to use as input (if not specified, use OAK color camera)")
parser_tracker.add_argument("--pd_model", type=str,
                    help="Path to a blob file for palm detection model")
parser_tracker.add_argument('--no_lm', action="store_true", 
                    help="Only the palm detection model is run (no hand landmark model)")
parser_tracker.add_argument("--lm_model", type=str,
                    help="Landmark model 'full', 'lite', 'sparse' or path to a blob file")
parser_tracker.add_argument('--use_world_landmarks', action="store_true", 
                    help="Fetch landmark 3D coordinates in meter")
parser_tracker.add_argument('-s', '--solo', action="store_true", 
                    help="Solo mode: detect one hand max. If not used, detect 2 hands max (Duo mode)")                    
parser_tracker.add_argument('-xyz', "--xyz", action="store_true", 
                    help="Enable spatial location measure of palm centers")
parser_tracker.add_argument('-g', '--gesture', action="store_true", 
                    help="Enable gesture recognition")
parser_tracker.add_argument('-c', '--crop', action="store_true", 
                    help="Center crop frames to a square shape")
parser_tracker.add_argument('-f', '--internal_fps', type=int, 
                    help="Fps of internal color camera. Too high value lower NN fps (default= depends on the model)")                    
parser_tracker.add_argument("-r", "--resolution", choices=['full', 'ultra'], default='full',
                    help="Sensor resolution: 'full' (1920x1080) or 'ultra' (3840x2160) (default=%(default)s)")
parser_tracker.add_argument('--internal_frame_height', type=int,                                                                                 
                    help="Internal color camera frame height in pixels")   
parser_tracker.add_argument("-lh", "--use_last_handedness", action="store_true",
                    help="Use last inferred handedness. Otherwise use handedness average (more robust)")                            
parser_tracker.add_argument('--single_hand_tolerance_thresh', type=int, default=10,
                    help="(Duo mode only) Number of frames after only one hand is detected before calling palm detection (default=%(default)s)")
parser_tracker.add_argument('--dont_force_same_image', action="store_true",
                    help="(Edge Duo mode only) Don't force the use the same image when inferring the landmarks of the 2 hands (slower but skeleton less shifted)")
parser_tracker.add_argument('-lmt', '--lm_nb_threads', type=int, choices=[1,2], default=2, 
                    help="Number of the landmark model inference threads (default=%(default)i)")  
parser_tracker.add_argument('-t', '--trace', type=int, nargs="?", const=1, default=0, 
                    help="Print some debug infos. The type of info depends on the optional argument.")                
parser_renderer = parser.add_argument_group("Renderer arguments")
parser_renderer.add_argument('-o', '--output', 
                    help="Path to output video file")
args = parser.parse_args()
dargs = vars(args)
tracker_args = {a:dargs[a] for a in ['pd_model', 'lm_model', 'internal_fps', 'internal_frame_height'] if dargs[a] is not None}

if args.edge:
    from HandTrackerEdge import HandTracker
    tracker_args['use_same_image'] = not args.dont_force_same_image
else:
    from HandTracker import HandTracker


tracker = HandTracker(
        input_src=args.input, 
        use_lm= not args.no_lm, 
        use_world_landmarks=args.use_world_landmarks,
        use_gesture=args.gesture,
        xyz=args.xyz,
        solo=args.solo,
        crop=args.crop,
        resolution=args.resolution,
        stats=True,
        trace=args.trace,
        use_handedness_average=not args.use_last_handedness,
        single_hand_tolerance_thresh=args.single_hand_tolerance_thresh,
        lm_nb_threads=args.lm_nb_threads,
        **tracker_args
        )

renderer = HandTrackerRenderer(
        tracker=tracker,
        output=args.output)

gesture_dir = {"FIVE" : 0, "ONE": 0, "TWO":0, "THREE":0, "FIST":0, "PEACE":0, "OK":0,"FOUR":0}
detect_thres = 30
while True:
    # Run hand tracker on next frame
    # 'bag' contains some information related to the frame 
    # and not related to a particular hand like body keypoints in Body Pre Focusing mode
    # Currently 'bag' contains meaningful information only when Body Pre Focusing is used
    frame, hands, bag = tracker.next_frame()
    if frame is None: break
    # Draw hands
    frame = renderer.draw(frame, hands, bag)
    for hand in hands:
#         print(hand.gesture)
        if hand.gesture in gesture_dir.keys():
            gesture_dir[hand.gesture] = gesture_dir[hand.gesture] + 1
    print(gesture_dir)
    if gesture_dir["FIST"]>detect_thres:
        runAction("boxing2.d6ac")
        gesture_dir["FIST"] = 0
    if gesture_dir["OK"]>detect_thres:
        runAction("nod.d6ac")
        gesture_dir["OK"] = 0
    if gesture_dir["ONE"]>detect_thres:
        runAction("pee.d6ac")
        gesture_dir["ONE"] = 0
    if gesture_dir["TWO"]>detect_thres:
        runAction("wave.d6ac")
        gesture_dir["TWO"] = 0
    if gesture_dir["PEACE"]>detect_thres:
        runAction("shake_hands.d6ac")
        gesture_dir["PEACE"] = 0
    if gesture_dir["THREE"]>detect_thres:
        runAction("shake_head.d6ac")
        gesture_dir["THREE"] = 0
    if gesture_dir["FIVE"]>detect_thres:
        runAction("spacewalk.d6ac")
        gesture_dir["FIVE"] = 0
        
    
        
        
        
    key = renderer.waitKey(delay=1)
    if key == 27 or key == ord('q'):
        break
renderer.exit()
tracker.exit()
