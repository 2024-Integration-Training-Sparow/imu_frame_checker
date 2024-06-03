IMUFrameCheckerパッケージはimu_linkの向きが正しく設定されているかを半自動で確かめます。
Autowareを起動したターミナルとは別のターミナルで

    ros2 run imu_frame_checker imu_frame_checker 
    
を実行する。
以下のように「機体を静止させてください」「機体を前進させてください」と指示が表示されるのでそれに従う。

![alt text](image.png)
![alt text](image-1.png)

以下のようにIMU frame is correctと表示されたら正しくimu_linkが設定されています。

![alt text](image-2.png)