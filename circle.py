#!/usr/bin/python

# CIRCLEモードで一周して自動着陸するスクリプト

import dronekit
import time
import math

from dronekit import Vehicle, VehicleMode, connect, LocationGlobal, LocationGlobalRelative

sequence = 0                # 動作シーケンス(0:～円周序盤 1:円周後半 2:円周終了)
targetAltitude = 20         # 円周飛行を開始する目標高度

# 位置情報が変更されたときに呼び出されるコールバック
def location_callback(self, attr_name, value):
    global sequence

    # 距離
    # distance = math.sqrt(value.north**2 + value.east**2)
    distance = get_distance_metres(start_location, vehicle.location.global_relative_frame)

    # 開始位置から遠ざかる状態→周りはじめの位置誤差による円周終了の誤判定を防ぐ
    if sequence == 0:
        if distance > 5:
            sequence = 1
        # print("距離:", distance)
    # 十分移動した後で、元の位置に戻る状態の判定
    elif sequence == 1:
        # print("距離:", distance)
        if 1.0 > distance:
            # 一周終了→離陸地点に帰る（RTLに入るのはメインルーチンで）
            sequence = 2

# 高度が変更されたときに呼び出されるコールバック
def altitude_callback(self, attr_name, value):
    global vehicle
    # print("高度:", value.alt)

# 機体に接続する
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation


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


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

# フライトモードを変更する
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")

# ARM可能か確認する
while not vehicle.is_armable:
    print("ARM可能になるのを待っています")
    time.sleep(1)

rc3 = vehicle.channels.get('3')
while 1000 < rc3:
    print("スロットルを最スローにしてください")
    time.sleep(1)
    rc3 = vehicle.channels.get('3')

# ARM実行
vehicle.arm()

# テイクオフ実行
print("テイクオフ！")
vehicle.simple_takeoff(targetAltitude)

# 高度監視のコールバック追加
vehicle.add_attribute_listener('location.global_relative_frame', altitude_callback)

# 目標高度到達まで待つループ
while True:
    if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
        print("目標高度に到達")
        break
    time.sleep(1)

print("円周開始地点に移動")
goto(10, 10)

rc3 = vehicle.channels.get('3')
while 1500 > rc3:
    print("スロットルをニュートラル以上にしてください")
    time.sleep(1)
    rc3 = vehicle.channels.get('3')

# Circleモードで円周飛行
vehicle.mode = VehicleMode("CIRCLE")
vehicle.wait_for_mode("CIRCLE")
print("円周飛行開始")

start_location = vehicle.location.global_relative_frame

# HPからの距離を監視するコールバック追加
vehicle.add_attribute_listener('location.local_frame', location_callback)

# Circle動作が一周するのを待つ
while sequence < 2:
    time.sleep(1)

# RTLモードに切り替える
vehicle.mode = VehicleMode("RTL")
vehicle.wait_for_mode("RTL")
print("円周飛行完了、帰還します")

# DISARMされるまで待ち続ける（位置情報の更新はコールバックで表示）
vehicle.disarm()
print("ミッション終了")
