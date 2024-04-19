
環境構築 2024/4/19

１．地図生成パッケージ：LIO-SAM-MID360

　■ 原始ソース：https://github.com/nkymzsy/LIO-SAM-MID360
 
　■ 依存関係とInstall方法：https://github.com/TixiaoShan/LIO-SAM/
 
２．自己位置推定パッケージ：hdl_localization

  ■ 原始ソース：https://github.com/koide3/hdl_localization
  
  ■ 依存関係とInstall方法：https://github.com/koide3/hdl_localization
  
  ※　依存パッケージ：pcl_ros、ndt_omp、fast_gicp、hdl_global_localization
  
3. MID360 LiDARドライバ：livox_ros_driver2
4. 
  ■ 原始ソース：https://github.com/Livox-SDK/livox_ros_driver2

  ■ Install方法と注意事項、パラメータ設定方法：https://github.com/Livox-SDK/livox_ros_driver2
  
    ※ちゃんと読んでください
    
４．ナビゲーションパッケージ：navigation

  ■　原始ソース：https://github.com/ros-planning/navigation
  
  ■ Install方法：
  
　　　- sudo apt install libbullet-dev libsdl-image1.2-dev libsdl-dev
   
     - sudo apt install ros-noetic-navigation
     
     - sudo apt install ros-noetic-geometry2
     
6. 2d地図生成パッケージ：gmapping
   
   - Install方法: sudo apt install ros-noetic-gmapping
     
8. ルート追従パッケージ：ROS_Pure_Pursuit
   
   ■原始コード：https://github.com/leofansq/ROS_Pure_Pursuit
   
   ■ ビルド、使用方法：https://github.com/leofansq/ROS_Pure_Pursuit
   
10. モータ駆動ソフトパッケージ：om_modbus_master
    
   ■ 仕様書：https://www.orientalmotor.com/support/ROS_Package_Technical_Material_ver_4.pdf

   ■ 原始ソース：https://www.orientalmotor.co.jp/ja/support/connection-partners
   「対象メーカ」 => 「ROS」＝＞「ROS1用Nodeom_modbus_master_V108」※登録必要 

/************2024/04/18**更新内容*******************************/
1. 自主計画ルート走行を入れた。
2. fast-lio2での地図生成パッケージを削除した
3. 地図データは「amr_ros/maps/」のフォルダに移動した。２D/３D地図
4. om_cartにバッテリ、速度情報を外に出す。モータのブレーキをつけ、消すサービス機能追加。
5. fast_gicpパッケージはローカルしたあと、thirdparty.zipを解凍すること。
   
///////////////////////////////////////////////////////////////

注意事項

1)fast_gicpのthirdparty.zipファイルを解凍すること

2)fast-lioパッケージ：①閉じ込めない問題有る　②生成地図データサイズ大き過ぎ　③地図生成時、振動有る場合、ソフト崩れやすいため、評価中止

3)各パッケージの変更内容は、原始ソースコードを比較すると見やすくなる

