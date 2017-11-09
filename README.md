# hsr_demo
##2017.11.11,12に開催のコスモス祭で使用するガバガバデモンストレーション

最初に、HSRを原点まで移動させる。この際、rvizで自己位置を修正しなくてはならない。
HSRはwifiをaibotに接続しておいたので、ホストPCのwifiの接続先を間違えないように。
`$hsrb_mode`
`$rosrun rviz rviz`
rvizが起動したら、addからrobot_modelとmapを追加しておく。マップのトピックは

(Tool Propertiesの2D Pose EstimateのTopicを /laser_2d_correct_pose に変更すること)

別のターミナルを開いて
`$hsrb_mode`
`$source ~/catkin_ws/devel/setup.bash`
`$rosrun hsr_demo hsr_demo.py`
これで動くはず。

デモが終わったら原点に戻るようになっているが、その後の説明などで自己位置がずれたらまたrvizで修正してください...。
