## 2017.11/11,12に開催のコスモス祭で使用するガバガバデモンストレーション

最初に、HSRを原点まで移動させる。この際、rvizで自己位置を修正しなくてはならない。  
HSRはwifiをaibotに接続しておいたので、**ホストPCのwifiの接続先を間違えないように。**

    $hsrb_mode
    $rosrun rviz rviz

rvizが起動したら、addからrobot_modelとmapを追加しておく。

紫で囲った場所あたりを右クリックして**ToolProperties**を選択。**2D Pose Estimate**のTopicを`/laser_2d_correct_pose`に変更すること  
![rviz](https://github.com/TANUKIpro/img_dock/blob/master/rviz.png)  

HSRのターミナルを開いて  
`$roslaunch bringup bringup.launch`
コメント:bringupはとても沢山のモジュールをインポートしなくてはならないので、事前にGithubからcloneしておくのがおすすめ  
ホストPCのターミナルを開いて  

    $hsrb_mode
    $source ~/catkin_ws/devel/setup.bash
    $rosrun hsr_demo hsr_demo.py
これで動くはず。  

デモが終わったら原点に戻るようなってるけど、その後の説明などで自己位置がずれたらまたrvizで修正してください...。
