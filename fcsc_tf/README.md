fcsc_tf
=============================

## はじめに
座標系

## 各ノードの説明
### virtual_joint_broadcaster

#### サブスクライブ・トピック
* cmd_vel (geometry_msgs/Twist)
* robot_pose (geometry_msgs/Pose2D)

### shelf_tf_broadcaster
####  サービス・サーバ
* update_shelf_pose (std_srvs/Empty)
クライアントから要求があるとRviz上の棚モデルから棚、棚板の座標系を出す．

#### サービス・クライアント
* get_planning_scene (moveit_msgs/GetPlanningScene)

### sandwich_tf_broadcaster
Rviz上のサンドイッチモデルを読み込んでサンドイッチの座標系を出す．

#### サービス・クライアント
* get_planning_scene (moveit_msgs/GetPlanningScene)

### grasped_object_tf_broadcaster
Rviz上でロボットが把持している物体(AttachedObject)の座標系を出す

#### サービス・クライアント
* get_planning_scene (moveit_msgs/GetPlanningScene)

### detected_object_tf_broadcaster
#### サブスクライブ・トピック
* /detected_object_array (csc_msgs/RecognizedObjectArray)
認識した物体の座標系を出す
