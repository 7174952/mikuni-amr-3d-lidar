# mikuni-amr-3d-lidar
2024/3/1

１．AMR手動操作方法：
  ＞台車給電、ubuntuのデスクトップで「amr_start.sh」スクリプトをダブルクリックして、台車のrs485-USBケーブルを検出する。※ダブルクリックしたあと、反応なくても問題ない
  ＞$roslaunch amr_ros om_manual.launch
  ＞joystickをつながってから、joyを操縦する。
  
２．地図(3D)生成方法(3D点群):
  ＞台車は上記1の手動操作方法通りに起動する
  ＞地図生成ソフト起動：$roslaunch amr_ros om_2d_liosam_map.launch
  ＞生成終了：ターミナルで「ctrl+c」、地図データ(点群)は自動的にlio-samフォルダ下のLOAMフォルダに保存する
  
３．地図(2D)生成方法(平面)
  ＞hdl_localizationパッケージのhdl_localization_livox.launchファイルに3D点群ファイル名を書き換える
  ＞3d-2D変換ソフトを起動する：$roslaunch amr_ros om_change_map_3d_to_2d.launch
  ＞2D地図変換待つ：3d-2d変換ソフトを起動したあと、rvizで2D地図が表示できるまで待つ
  ＞２D地図保存：$roslaunch amr_ros om_save_2d_map.launch

注意事項：

1)fast_gicpのthirdparty.zipファイルを解凍すること

2)ROS_Pure_Pursuitのpure_pursuit.cppファイルに180度旋回と目的地に障害物ある場合、目的地に着いて行けない不具合を修正した

3)fast-lioパッケージ：①閉じ込めない問題有る　②生成地図データサイズ大き過ぎ　③地図生成時、振動有る場合、ソフト崩れやすいため、評価中止

