fcsc_moveit
======================

## はじめに

### ノードの説明
#### smach_daihen_ur5_moveit_core
##### サービス・サーバ
* detect_product (fcsc_msgs/DetectProduct)  
陳列する商品の情報を取得して、商品のモデルをRviz上に出す
</br>

* detect_shelf_action (fcsc_msgs/DetectShelf)  
ロボットアームを動かして陳列棚に貼り付けたマーカを見て陳列棚のモデルをRviz上に出す
</br>


* detect_sandwich (fcsc_msgs/DetectSandwich)  
ロボットアームを動かしてサンドイッチに貼り付けたマーカを見て認識したサンドイッチの情報を取得する．　　
認識したサンドイッチをRviz上に出す．
</br>

* pickup_product (fcsc_msgs/Manipulate)  
コンテナに格納されている商品をピックアップする．
</br>

* place_product (fcsc_msgs/Manipulate)  
陳列棚に商品を陳列する．
</br>

* return_product (fcsc_msgs/Manipulate)  
把持した商品をコンテナに戻す．
</br>

* pickup_sandwich (fcsc_msgs/Manipulate)  
サンドイッチをピックアップする．
</br>

* faceup_standing_sandwiches (fcsc_msgs/Manipulate)  
陳列棚内で立っているサンドイッチ(底面が接地している状態)を指定された場所に並べる
</br>

* recover_sandwich (fcsc_msgs/Manipulate)  
ピックアップしたサンドイッチをコンテナに格納する
</br>

* detect_and_recover_sandwich (fcsc_msgs/DetectAndRecoverSandwich)  
ロボットアームを動かしてサンドイッチを認識する．
認識したらサンドイッチをピックアップしてコンテナに格納する．
</br>

* change_sandwich_pose (fcsc_msgs/Manipulate)  
倒れているサンドイッチを立たせる
</br>

* move_initial_pose (std_srvs/Trigger)  
ロボットアームを初期姿勢に移動させる
</br>

* goto_stocking_base_position (std_srvs/Trigger)  
陳列作業をするための作業場所に移動する
</br>

* goto_faceup_base_position (std_srvs/Trigger)  
フェイスアップ・回収作業をするための作業場所に移動する
</br>

* goto_near_shelf (fcsc_msgs/GoToNearShelf)  
陳列棚の近くに移動する
</br>

* goto_home_position (std_srvs/Trigger)  
ホームポジションに移動する
</br>

* sort_order_picking_sandwiches (fcsc_msgs/SortOrderManipulation)  
サンドイッチのマニピュレーションの順番を決める

##### サービス・クライアント
* get_planning_scene (moveit_msgs/GetPlanningScene)
* compute_ik (moveit_msgs/GetPositionIK)
* detect_object (fcsc_msgs/DetectObject)
* detect_shelf (fcsc_msgs/DetectObject)
* /link_attacher_node/attach (gazebo_ros_link_attacher/Attach)
* /link_attacher_node/detach (gazebo_ros_link_attacher/Attach)

##### パブリッシュ・トピック
* /move_group/display_planned_path (moveit_msgs/DisplayTrajectory)
* /shelf_detector/ar_track_alvar/enable_detection (std_msgs/Bool)
* /ar_track_alvar/enable_detection (std_msgs/Bool)

##### サブスクライブ・トピック
* /execute_trajectory/result (moveit_msgs/ExecuteTrajectoryActionResult)
* time_finish (std_msgs/Empty)

#### minute_timer
タイマーノード  
ノート立ち上げ時にparamで制限時間(分)を設定．
制限時間経過後に通知する．

##### パブリッシュ・トピック
* time_finish (std_msgs/Empty)  
制限時間経過後にパブリッシュする．
