fcsc_perception
=============================

## はじめに
物体認識用パッケージ  

## 各ノードの説明
### detect_object_server
認識した物体を管理するサービスサーバ

#### サービスサーバ
* detect_object (fcsc_msgs/DetectObject)  
クライアントに要求に対して、以下の処理を行う
  * 物体認識
  * 認識済み物体の情報を全削除
  * 指定した物体の情報を削除　
  * 認識した物体をRviz上に出す

#### サービスクライアント
* get_object (fcsc_msgs/DetectObject)  

#### パブリッシュ・トピック
* /detected_object_array (fcsc_msgs/RecognizedObjectArray)  

#### サブスクライブ・トピック
* /object_detection_node/detected_object_array (fcsc_msgs/RecognizedObjectArray)  

### ar_pose_to_object_pose_server  
ar_track_alvar(ARマーカ認識ノード)から受け取ったマーカ位置をサンドイッチの位置に変換し、物体情報(位置・姿勢など)を送信するサービスサーバ

#### サービスサーバ
* get_object (fcsc_msgs/DetectObject)  
クライアントから要求があると、認識したサンドイッチ情報を送信する

#### パブリッシュ・トピック
* /ar_track_alvar/enable_detection (std_msgs/Bool)

#### サブスクライブ・トピック
* ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers)

### shelf_detector

#### サービスサーバ
* detect_shelf (fcsc_msgs/DetectObject)

#### パブリッシュ・トピック
* shelf_detector/ar_track_alvar/enable_detection (std_msgs/Bool)

#### サブスクライブ・トピック
* shelf_detector/ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers)

### estimate_object_position
