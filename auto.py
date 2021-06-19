#!/usr/bin/python

# Waypointに従って自動飛行させるスクリプト
# 前提条件：あらかじめ waypoint が設定されていること、 waypoint の最後に RTL が含まれること

import dronekit
import time
import math

from dronekit import Vehicle, VehicleMode, connect
from pymavlink.dialects.v10.ardupilotmega import MAVLink_mission_request_list_message
from test.support import missing_compiler_executable

# 位置情報が変更されたときに呼び出されるコールバック
def location_callback(self, attr_name, value):
    distance = math.sqrt(value.north**2 + value.east**2)
    print("距離:", distance)

# 高度が変更されたときに呼び出されるコールバック
def altitude_callback(self, attr_name, value):
    print("高度:", value.alt)

# 機体と接続する
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# フライトプランを読み出してRTLが含まれているか確認する
isExistRTL=0
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
missionList=[]
for cmd in cmds:
    missionList.append(cmd)
for cmd in missionList:
    if cmd.command == 20:
        isExistRTL=1
        break
    # print("コマンド", cmd)

# RTLが含まれていない場合は強制終了
if isExistRTL==0:
    print("フライトプランの最後に RETURN_TO_LAUNCH を追加してください")
    exit()

# フライトモードをGUIDEDに変更（スクリプトでの離陸用）
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")

# ARMできるまで待つループ
while not vehicle.is_armable:
    print("ARM可能になるのを待っています")
    time.sleep(1)

# ARM実行
vehicle.armed = True
vehicle.arm()

# 離陸→目標高度まで上昇開始
targetAltitude = 20
print("テイクオフ")
vehicle.simple_takeoff(targetAltitude)

# 高度監視のコールバック追加
vehicle.add_attribute_listener('location.global_relative_frame', altitude_callback)

# 目標高度到達まで待つループ
while True:
    if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
        print("目標高度に到達")
        break
    time.sleep(1)

# Waypointに従って飛行開始
print("Waypoint飛行開始")
vehicle.mode = VehicleMode("AUTO")
vehicle.wait_for_mode("AUTO")

# HPからの距離を監視するコールバック追加
vehicle.add_attribute_listener('location.local_frame', location_callback)

# DISARMされるまで待ち続ける（位置情報の更新はコールバックで表示）
vehicle.disarm()
print("ミッション終了")
