$roslaunch robot_description display.launnch
で、rvizとgazeboが起動してロボットモデルが表示されます。

$roslaunch robot_description navigation.launch
で move_base を起動して、

$roslaunch comm_clisent Comm_Simulation.launch
で comm_simulation が起動して、サービスによって自己位置要求・移動指令・ロボット状態要求が行えます。
サービスの内容は comm_client フォルダ内の"通信サービス.pdf"を参照してください。