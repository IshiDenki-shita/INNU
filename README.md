
## 概要
これはPBL課題に関するソースコードをまとめたリポジトリです。

ラズベリーパイを2台使ってロボットを制御することを目的とする。

LiDARラズパイ：LiDARとモーター制御を担当。
画像ラズパイ：画像認識を行い目標地点を把握する。

これらはUARTで通信し合う。

### 動作の方法

## 環境

（共通）
・L298N     ※モータードライバ

（LiDARラズパイ）
・Raspberry Pi 3B+
・ubuntu 18.04.7 LTS
・bionic

・RPLiDAR A1M8-R6

python3.11.11を使ってください
ubuntuのバージョンの影響で、Pythonは手動でインストールする必要があります。

（画像ラズパイ）
・raspberry pi 3B+
・ラズパイOS

・Raspberry pi camera module 2

picamera2はラズパイOSで動きます。Macで動きません。

## 使用したもの

### LiDARラズパイ
・pigioの (zipファイル)
https://github.com/joan2937/pigpio/archive/master.zip

・ROSに関するリポジトリ（クローン）
https://github.com/Slamtec/rplidar_ros.git

・その他の使用ライブラリはrequirements.txtを参照してください。

### 画像ラズパイ