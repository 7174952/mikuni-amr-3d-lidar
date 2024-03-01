# mikuni-amr-3d-lidar
2024/3/1

１．AMR手動操作方法：
  ＞台車給電、ubuntuのデスクトップで「amr_start.sh」スクリプトをダブルクリックして、台車のrs485-USBケーブルを検出する。※ダブルクリックしたあと、反応なくても問題ない
  ＞$roslaunch amr_ros om_manual.launch
  ＞joystickをつながってから、joyを操縦する。
２．地図生成方法(3D点群):
  ＞台車は上記1の手動操作方法通りに起動する
  ＞地図生成ソフト起動：$roslaunch amr_ros om_2d_liosam_map.launch
  ＞生成終了：ターミナルで「ctrl+c」、地図データ(点群)は自動的に

注意事項：

1)fast_gicpのthirdparty.zipファイルを解凍すること

2)ROS_Pure_Pursuitのpure_pursuit.cppファイルに180度旋回と目的地に障害物ある場合、目的地に着いて行けない不具合を修正した

3)fast-lioパッケージ：①閉じ込めない問題有る　②生成地図データサイズ大き過ぎ　③地図生成時、振動有る場合、ソフト崩れやすいため、評価中止

