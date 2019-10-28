<fcsc_arm.xacro と turtlebot_gripper.xacro の取り付けについて>

fcsc_arm.xacro の手首リンク link6 の長さを変更したとき、
turtlebot_gripper.xacro と fcsc_gazebo.xacro の以下の場所を変更する。

link6 をx[m]に変更した場合
  turtlebot_gripper.xacro
  ・<joint name="gripper_link_joint" type="fixed"> 内の
  <origin xyz="${0.1850+0.1-x} 0 0" rpy="0 0 0" />

  ・<bioloid_F3_fixed parent="link6" name="arm_wrist_F3_0" color="${color}">内の
  <origin xyz="x 0 0" rpy="${M_PI/2} 0 ${-M_PI/2}" />

  ・<dynamixel_AX12_fixed parent="${parent}" name="gripper_servo">内の
  <origin xyz="${x + AX12_WIDTH/2} -0.00675 0" rpy="${M_PI/2} 0 0"/>

  fcsc_gazebo.xacro
  ・<joint name="kinect_anc_joint" type="fixed">内の
  <origin xyz="0 0 ${x/2}" rpy="0 0 0"/>


<商品モデルファイルの説明>
*_original.urdf.xacro:*.stlを使用　物体原点位置は物体の中心に設定
＊.urdf.xacro:*_edit.stlを使用 物体原点位置は特にいじらず
