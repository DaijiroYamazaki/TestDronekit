#!/usr/bin/python

# CIRCLEモードで一周して自動着陸するスクリプト

import dronekit
import time
import math

from dronekit import Vehicle, VehicleMode, connect, LocationGlobal, LocationGlobalRelative
from pymavlink.generator.mavgen_python import byname_hash_from_field_attribute

sequence = 0                # 動作シーケンス(0:～円周序盤 1:円周後半 2:円周終了)
targetAltitude = 20         # GUIDED による移動開始高度

# 位置情報が変更されたときに呼び出されるコールバック
def circle_location_callback(self, attr_name, value):
    global sequence

    # RC3：スロットルをニュートラルにする
    # @todo 実機でRCを使う場合はこの行を削除すること
    vehicle.channels.overrides = {'3':1500}

    # 距離
    # distance = math.sqrt(value.north**2 + value.east**2)
    distance = get_distance_metres(start_location, vehicle.location.global_frame)
    print("円周開始位置からの距離:", distance)

    # 開始位置から遠ざかる状態→周りはじめの位置誤差による円周終了の誤判定を防ぐ
    if sequence == 0:
        if distance > 5:
            sequence = 1
        # 
    # 十分移動した後で、元の位置に戻る状態の判定
    elif sequence == 1:
        if 5.0 > distance:
            # 一周終了→離陸地点に帰る（RTLに入るのはメインルーチンで）
            sequence = 2
            vehicle.mode = VehicleMode("GUIDED")

# 機体に接続する
test_vehicle = { 
    connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60),
    connect('tcp:127.0.0.1:5772', wait_ready=True, timeout=60),
    connect('tcp:127.0.0.1:5782', wait_ready=True, timeout=60),
    connect('tcp:127.0.0.1:5792', wait_ready=True, timeout=60),
    connect('tcp:127.0.0.1:5802', wait_ready=True, timeout=60)
}

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


target_location = LocationGlobal(lat=35.806627, lon=139.08252, alt=500)

# 直近の機体を検索する
shortest = 999999
for v in test_vehicle:
    distance = get_distance_metres(v.location.global_frame, target_location)
    if distance < shortest:
        shortest = distance
        vehicle = v

print("vehicle_pos: ", vehicle.location.global_frame)

# GUIDED モード
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")

return_pos = vehicle.location.global_frame
return_pos.alt = targetAltitude

# ARM可能か確認する
while not vehicle.is_armable:
    print("ARM可能になるのを待っています")
    time.sleep(1)

# ARM実行
vehicle.arm()

# 飛行開始時刻記録
start_time = time.clock_gettime(0)

# テイクオフ実行
print("テイクオフ！")
vehicle.simple_takeoff(targetAltitude)

# 目標高度到達まで待つループ
while True:
    time.sleep(1)
    print("高度:",  vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= targetAltitude - 5:
        print("目標高度に到達")
        break

print("円周開始地点に移動")
vehicle.simple_goto(target_location, airspeed=200, groundspeed=200)

# 目標地点到達による速度ゼロを待つループ
while True:
    time.sleep(3)
    vx = vehicle.velocity[0]
    vy = vehicle.velocity[1]
    velocity = math.sqrt( vx*vx + vy*vy )
    print("速度", velocity)
    if 1.0 > velocity:
        break

# RC3：スロットルをニュートラルにする
# @todo 実機でRCを使う場合はこの行を削除すること
vehicle.channels.overrides = {'3':1500}

# @todo 実機でRCを使う場合の処理：スロットルニュートラルを待つ
# rc3 = vehicle.channels.get('3')
# while 1500 > rc3:
#     print("スロットルをニュートラル以上にしてください")
#     time.sleep(1)
#     rc3 = vehicle.channels.get('3')

# Circleモードで円周飛行
vehicle.mode = VehicleMode("CIRCLE")
vehicle.wait_for_mode("CIRCLE")
print("円周飛行開始")

start_location = vehicle.location.global_frame

# 円周開始位置からの距離を監視するコールバック追加
vehicle.add_attribute_listener('location.local_frame', circle_location_callback)

# Circle動作が一周するのを待つ
while sequence < 2:
    time.sleep(1)

# コールバックが GUIDED モードに切り替えているはずなので確認
vehicle.wait_for_mode("GUIDED")

# 円周開始位置からの距離を監視するコールバック削除
vehicle.remove_attribute_listener('location.local_frame', circle_location_callback)

# RTLモードに切り替える
print("円周飛行完了、帰還します")
vehicle.simple_goto(return_pos, airspeed=200, groundspeed=200)

# 目標地点到達による速度ゼロを待つループ
while True:
    time.sleep(3)
    vx = vehicle.velocity[0]
    vy = vehicle.velocity[1]
    velocity = math.sqrt( vx*vx + vy*vy )
    print("速度", velocity)
    if 1.0 > velocity:
        break

vehicle.mode = VehicleMode("RTL")
vehicle.wait_for_mode("RTL")

# DISARMされるまで待ち続ける（位置情報の更新はコールバックで表示）
vehicle.disarm(wait=True, timeout=None)
print("ミッション終了")

# 帰還時刻記録
finish_time = time.clock_gettime(0)

# 飛行時間報告
flight_time = finish_time - start_time
print("飛行時間:", flight_time)
