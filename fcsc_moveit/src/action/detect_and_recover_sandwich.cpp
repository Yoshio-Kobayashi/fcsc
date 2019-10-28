#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"


bool FcscCore::detectAndRecoverSandwich(fcsc_msgs::DetectAndRecoverSandwich::Request &req, fcsc_msgs::DetectAndRecoverSandwich::Response &res)
{
  // 手前側認識動作でサンドイッチ認識開始
  int detect_pattern = 1;
  int max_detect_pattern = 4;

  std::vector<ScrapSandwichInfo> scrap_sandwich_infos(scrap_sandwich_ids.size());

  // 廃棄サンドイッチの情報を初期化
  for (size_t i = 0; i < scrap_sandwich_ids.size(); i++) {
    scrap_sandwich_infos[i].name = "sandwich_" + scrap_sandwich_ids[i];
    scrap_sandwich_infos[i].picking_count = 0;
  }

  while (1) {
    fcsc_msgs::DetectObject detect_object_srv;

    // 1.サンドイッチ認識動作
    fcsc_msgs::DetectSandwich detect_sandwich_srv;
    detect_sandwich_srv.request.pattern = detect_pattern;
    // detect_sandwich_srv.request.demo = true;
    ROS_WARN("[detectAndRecoverSandwich]:detectSandwich");
    detectSandwich(detect_sandwich_srv.request, detect_sandwich_srv.response);

    // 2.認識した廃棄サンドイッチをvectorの先頭にまとめる
    // 廃棄品の回収を優先させる
    std::vector<fcsc_msgs::RecognizedObject> scrap_sandwiches;
    std::vector<fcsc_msgs::RecognizedObject> normal_sandwiches;
    for (size_t i = 0; i < detect_sandwich_srv.response.sandwiches.size(); i++) {
      fcsc_msgs::RecognizedObject sandwich = detect_sandwich_srv.response.sandwiches[i];
      sandwich.scrap == true ? scrap_sandwiches.push_back(sandwich) : normal_sandwiches.push_back(sandwich);
    }

    // 通常のサンドイッチをレスポンスに格納しておく
    for (size_t i = 0; i < normal_sandwiches.size(); i++) {
      int normal_index = -1;
      for (size_t j = 0; j < res.normal_sandwiches.size(); j++) {
        if (res.normal_sandwiches[j].name == normal_sandwiches[i].name) {
          normal_index = j;
          break;
        }
      }
      if (normal_index >= 0) {
        res.normal_sandwiches[normal_index] = normal_sandwiches[i];
      } else {
        res.normal_sandwiches.push_back(normal_sandwiches[i]);
      }
    }

    if (scrap_sandwiches.size() > 0) { // 廃棄サンドイッチを認識した
      ROS_WARN("[detectAndRecoverSandwich]:Detect Success");

      // 認識した廃棄サンドイッチで規定回数以上ピッキングしていたものをカウント
      int picking_overflow_count = 0;
      for (size_t i = 0; i < scrap_sandwiches.size(); i++) {
        // index 選択
        for (size_t j = 0; j < scrap_sandwich_ids.size(); j++) {
          if (scrap_sandwich_infos[j].name == scrap_sandwiches[i].name) {
            if (scrap_sandwich_infos[j].picking_count >= 2) {
              picking_overflow_count++;
            }
            break;
          }
        }
      }
      // 今回の認識動作で認識した廃棄サンドイッチがすべて規定回数以上ピッキングしていた
      if (picking_overflow_count == scrap_sandwiches.size()) {
        ROS_WARN("[detectAndRecoverSandwich]:Picking overflow");
        if (detect_pattern != max_detect_pattern) {// まだ動作パターンある
          ROS_WARN("[detectAndRecoverSandwich]:Do next motion pattern");
          detect_pattern++;
          for (size_t i = 0; i < scrap_sandwiches.size(); i++) {
            detect_object_srv.request.delete_object_names.push_back(scrap_sandwiches[i].name);
          }
          detect_object_client.call(detect_object_srv);
          continue;
        } else {// 動作パターンない
          ROS_WARN("[detectAndRecoverSandwich]:No next motion pattern");
          ROS_WARN("[detectAndRecoverSandwich]:FALSE");
          res.success = false;
          return (true);
        }
      }

      // 3.サンドイッチを棚の手前優先で並べる
      fcsc_msgs::SortOrderManipulation sort_order_srv;
      sort_order_srv.request.objects = scrap_sandwiches;
      sortOrderPickingSandwiches(sort_order_srv.request, sort_order_srv.response);

      // 4.手前から順にサンドイッチ廃棄
      for (size_t i = 0; i < sort_order_srv.response.objects.size(); i++) {
        // index 選択
        int index;
        for (size_t j = 0; j < scrap_sandwich_ids.size(); j++) {
          if (scrap_sandwich_infos[j].name == sort_order_srv.response.objects[i].name) {
            index = j;
            break;
          }
        }

        if (!scrap_sandwich_infos[index].detected) {// 未検出
          // 検出済みに設定
          ROS_WARN("[detectAndRecoverSandwich]:First detected");
          scrap_sandwich_infos[index].detected = true;
        } else if (scrap_sandwich_infos[index].recoverd) {// 検出済み 廃棄済み(サンドイッチを掴めていなかった)
          // 未廃棄に設定
          ROS_WARN("[detectAndRecoverSandwich]:Detected Recovered");
          scrap_sandwich_infos[index].recoverd = false;
        } else {// 検出済み 未廃棄(過去にピッキングを試みて失敗した)
          ROS_WARN("[detectAndRecoverSandwich]:Detected Not recovered");
        }

        fcsc_msgs::Manipulate manipulate_srv;
        manipulate_srv.request.manipulation_type = fcsc_msgs::Manipulate::Request::RECOVER;
        manipulate_srv.request.object = sort_order_srv.response.objects[i];
        ROS_WARN("[detectAndRecoverSandwich]:pickupSandwich");
        pickupSandwich(manipulate_srv.request, manipulate_srv.response);
        scrap_sandwich_infos[index].picking_count++;
        if (!manipulate_srv.response.success) {
          ROS_WARN("[detectAndRecoverSandwich]:pickup false");
          continue;
        }
        ROS_WARN("[detectAndRecoverSandwich]:pickup success");
        recoverSandwich(manipulate_srv.request, manipulate_srv.response);
      }
    } else {//検出失敗
      ROS_WARN("[detectAndRecoverSandwich]:Detect False");
      if (detect_pattern != max_detect_pattern) {// 手前認識動作で廃棄サンドイッチを認識せず
        ROS_WARN("[detectAndRecoverSandwich]:Do next motion pattern");
        // 6.奥側動作でサンドイッチ認識に切り替え
        // 動作パターン変更
        detect_pattern ++;
      } else {// 奥認識動作で廃棄サンドイッチ認識せず
        ROS_WARN("[detectAndRecoverSandwich]:No next motion pattern");
        int recover_success_count = 0;
        for (size_t i = 0; i < scrap_sandwich_infos.size(); i++) {
          if (scrap_sandwich_infos[i].recoverd) {
            recover_success_count++;
          }
        }
        if (recover_success_count == scrap_sandwich_ids.size()) {// すべて廃棄済みなら成功
          ROS_WARN("[detectAndRecoverSandwich]:All recoverd");
          ROS_WARN("[detectAndRecoverSandwich]:SUCCESS");
          res.success = true;
        } else {// 廃棄済みでないなら失敗
          ROS_WARN("[detectAndRecoverSandwich]:FALSE");
          res.success = false;
        }
        asyncMoveArm(recover_pose_name);
        return (true);
      }
    }

    // 認識動作で認識したサンドイッチは消す
    for (size_t i = 0; i < scrap_sandwiches.size(); i++) {
      detect_object_srv.request.delete_object_names.push_back(scrap_sandwiches[i].name);
    }
    detect_object_client.call(detect_object_srv);

  }
}
