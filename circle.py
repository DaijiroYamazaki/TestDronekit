#!/usr/bin/python

# CIRCLEモードで一周して自動着陸するスクリプト

import dronekit
import time
import math

from dronekit import Vehicle, VehicleMode, connect

sequence = 0                # 動作シーケンス(0:～円周序盤 1:円周後半 2:円周終了)
targetAltitude = 20         # 円周飛行を開始する目標高度

# 位置情報が変更されたときに呼び出されるコールバック
def location_callback(self, attr_name, value):
    global sequence

    # RC3：スロットルをニュートラルにする
    # @todo 実機でRCを使う場合はこの行を削除すること
    vehicle.channels.overrides = {'3':1500}

    # 距離
    distance = math.sqrt(value.north**2 + value.east**2)
    print("距離:", distance)

    # 開始位置から遠ざかる状態→周りはじめの位置誤差による円周終了の誤判定を防ぐ
    if sequence == 0:
        if distance > 5:
            sequence = 1
    # 十分移動した後で、元の位置に戻る状態の判定
    else:
        if 0.5 > distance:
            # 一周終了→離陸地点に帰る（RTLに入るのはメインルーチンで）
            sequence = 2

# 高度が変更されたときに呼び出されるコールバック
def altitude_callback(self, attr_name, value):
    global vehicle
    print("高度:", value.alt)

# 機体に接続する
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)

# フライトモードを変更する
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")

# ARM可能か確認する
while not vehicle.is_armable:
    print("ARM可能になるのを待っています")
    time.sleep(1)

# ARM実行
vehicle.armed = True
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

# Circleモードで円周飛行
vehicle.mode = VehicleMode("CIRCLE")
vehicle.wait_for_mode("CIRCLE")
print("円周飛行開始")
vehicle.channels.overrides = {'3':1500}

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
