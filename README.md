# robot_map_compress_tool
在机器狗上程序运行后，在上位机开启以下命令：

ros2 run robot_map_encoder_decoder robot_decoder "robot_name"

接受压缩后的地图，发送ros2 topic: /robot_name/normal_map


注：
关于以上命令中的robot_name
2号机器狗默认为  tb1
26号机器狗默认为  tb0
