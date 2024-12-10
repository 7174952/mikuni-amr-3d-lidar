2024/12/10 GUI更新、停止角度制御修正

１．アプリとしてデスクトップで起動できた

$cd ~/catkin_ws/src/amr_ros/scripts/

$sudo cp GuideRobot.desktop /usr/share/applications/

2. ルート制御手順変更、gui操作変更
   
以前：手動走行１回目 - key waypointsを取得、手動走行2回目-期待ルート作成

変更後：手動走行１回目のみ - key waypointsと期待ルートを同時に取得する

３．自律走行停止時に向き制御できない不具合修正した
***************************************************************************************************
2024/12/3 音声制御、会話機能追加

１．python3.9の模擬環境必要(osはpython3.8を使う：/homeフォルダにmyenv3.9のフォルダに保存

前提条件：python3.9はすでにインストール済み状態

※音声制御のpythonシナリオはpython3.9を使う

$ sudo apt update

$ sudo apt install -y python3.9 python3.9-venv python3.9-dev

//create env

$ python3.9 -m venv myenv3.9

//active python env, 模擬環境に入る

$ source myenv3.9/bin/activate

2024/11/27 画像認識、先導機能追加
1. RealSenseカメラに向けの認識認識と距離測定機能追加した。pythonでのros node作成した
2. 先導機能に対して後ろの人間距離によって、ロボット速度を円滑に調整するように改善した
3. カメラの映像はrvizでリアルタイムで映ることに変えた。
********************************************************************************************************
2024/11/20 GUI処理機能追加

１．gui画面で地図生成できた

２．gui画面で指定ルート生成できた

３．gui画面で手動で走行シナリオ作成できた

４．gui画面操作で自律走行できた

注意：a.MID360 LidarのIPは正しいIP変更必要。場所：livox_ros_driver2/config/MID360_config.json

     b.テストは不十分のため、条件によって動かないや、ソフト崩れるなど場合がある。継続更新する予定。

NOTE: 2024/10/25 Add log function

・やり方：1) 保存フォルダ作成：/home/mikuni/cart_log

        2) amr_ros/nodes/save_cart_log.cpp：ローカルＰＣで同じフォルダに入れて、システムをリビルトすること
        
        3) 以前と同じでamr関連Lanuchファイルを起動
        
        4) 追加で新規ノード実行、ターミナル: $rosrun amr_ros save_cart_lo
        
        5）ログ保存場所でログを確認する。場所：/home/mikuni/cart_log
        
        6）内容は分かりづらい場合、excelで開いて、”；”で分割すること

**************************************************************************************************************
環境構築 2024/4/19

１．地図生成パッケージ：LIO-SAM-MID360

　■ 原始ソース：https://github.com/nkymzsy/LIO-SAM-MID360
 
　■ 依存関係とInstall方法：https://github.com/TixiaoShan/LIO-SAM/
 
２．自己位置推定パッケージ：hdl_localization

  ■ 原始ソース：https://github.com/koide3/hdl_localization
  
  ■ 依存関係とInstall方法：https://github.com/koide3/hdl_localization
  
  ※　依存パッケージ：pcl_ros、ndt_omp、fast_gicp、hdl_global_localization
  
3. MID360 LiDARドライバ：livox_ros_driver2
   
  ■ 原始ソース：https://github.com/Livox-SDK/livox_ros_driver2

  ■ Install方法と注意事項、パラメータ設定方法：https://github.com/Livox-SDK/livox_ros_driver2
  
    ※ちゃんと読んでください
    
４．ナビゲーションパッケージ：navigation

  ■　原始ソース：https://github.com/ros-planning/navigation
  
  ■ Install方法：
  
　　　- sudo apt install libbullet-dev libsdl-image1.2-dev libsdl-dev
   
     - sudo apt install ros-noetic-navigation
     
     - sudo apt install ros-noetic-geometry2
     
5. 2d地図生成パッケージ：gmapping
   
   - Install方法: sudo apt install ros-noetic-gmapping
     
6. ルート追従パッケージ：ROS_Pure_Pursuit
   
   ■原始コード：https://github.com/leofansq/ROS_Pure_Pursuit
   
   ■ ビルド、使用方法：https://github.com/leofansq/ROS_Pure_Pursuit
   
7. モータ駆動ソフトパッケージ：om_modbus_master
    
   ■ 仕様書：https://www.orientalmotor.com/support/ROS_Package_Technical_Material_ver_4.pdf

   ■ 原始ソース：https://www.orientalmotor.co.jp/ja/support/connection-partners
   「対象メーカ」 => 「ROS」＝＞「ROS1用Nodeom_modbus_master_V108」※登録必要 
**************************************************************************************************************
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

4)ソースRebuildでのエラーの解決方法：rebuild　＝＞ build　※rebuildは1回して、buildは4回繰り返したら、エラーがなくなる。

5)usb-rs485ケーブルはpcで自動に認識できなければ、次のシナリオを実施すること： catkin_ws/src/amr_ros/scripts/amr_start.sh
