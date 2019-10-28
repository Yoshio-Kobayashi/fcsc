#!/usr/bin/env python
import rospy
import roslib
import math
import tf
import smach
import smach_ros
from smach import Sequence, Iterator, StateMachine
from smach_ros import ServiceState, SimpleActionState
roslib.load_manifest('fcsc_msgs')
from fcsc_msgs.srv import *
from std_srvs.srv import Trigger

def main():
    rospy.init_node('fcsc_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    sm.userdata.products = []
    sm.userdata.sandwiches = []
    sm.userdata.scrap_sandwiches = []
    sm.userdata.normal_sandwiches = []
    sm.userdata.damy_output = []
    sm.userdata.adjust_count = 0;
    sm.userdata.faceup_count = 0;

    # Open the container
    with sm:
        def move_initial_pose_response_cb(userdata, response):
            if response.success == True:
                return 'succeeded'
            else:
                return 'aborted'

        smach.StateMachine.add('MOVE_INITIAL_POSE',
                                ServiceState('move_initial_pose',
                                              Trigger,
                                              response_cb=move_initial_pose_response_cb),
                                              # transitions={'succeeded':'DETECT_AND_RECOVER_SANDWICH'})
                                              transitions={'succeeded':'NAVIGATE_TO_SHELF_A'})

        navigate_to_shelf_a_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

        with navigate_to_shelf_a_sm:
            navigate_to_shelf_a_sm.add('GO_TO_NEAR_SHELF',
                                        ServiceState('goto_near_shelf', GoToNearShelf, request=GoToNearShelfRequest(0)),
                                        transitions={'succeeded':'DETECT_SHELF_A'})

            navigate_to_shelf_a_sm.add('DETECT_SHELF_A',
                                        ServiceState('detect_shelf_action', DetectShelf, request=DetectShelfRequest(0)),
                                        transitions={'succeeded':'GO_TO_STOCKING_POSITION', 'aborted':'aborted'})

            navigate_to_shelf_a_sm.add('GO_TO_STOCKING_POSITION',
                                        ServiceState('goto_stocking_base_position', Trigger),
                                        transitions={'succeeded':'succeeded'})


        smach.StateMachine.add('NAVIGATE_TO_SHELF_A',
                                navigate_to_shelf_a_sm,
                                # transitions={'succeeded':'NAVIGATE_TO_SHELF_B'})
                                transitions={'succeeded':'DETECT_PRODUCT'})

        smach.StateMachine.add('DETECT_PRODUCT',
                                ServiceState('detect_product',
                                            DetectProduct,
                                            response_slots=['detected_object_array']),
                                transitions={'succeeded':'STOCK_PRODUCTS'},
                                remapping={'detected_object_array':'products'})

        stock_it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                            input_keys=['products', 'damy_output'],
                            it=lambda: range(0, len(sm.userdata.products.objects)),
                            output_keys=['damy_output'],
                            it_label='index',
                            exhausted_outcome='succeeded')

        with stock_it:
            stock_sm = smach.StateMachine(outcomes=['continue', 'succeeded', 'preempted', 'aborted'],
                                          input_keys=['index', 'products'])
            with stock_sm:
                @smach.cb_interface(input_keys=['products', 'index'])
                def pickup_request_cb(userdata, request):
                    request.object = userdata.products.objects[userdata.index]
                    return request

                @smach.cb_interface(outcomes=['succeeded', 'aborted'])
                def pickup_response_cb(userdata, response):
                    if response.success == True:
                        return 'succeeded'
                    else:
                        return 'aborted'

                smach.StateMachine.add('PICKUP_PRODUCT',
                                        ServiceState('pickup_product',
                                                      Manipulate,
                                                      request_cb=pickup_request_cb,
                                                      response_cb=pickup_response_cb,
                                                      input_keys=['products', 'index']),
                                        transitions={'succeeded':'PLACE_PRODUCT', 'aborted':'continue'})

                @smach.cb_interface(input_keys=['products', 'index'])
                def place_request_cb(userdata, request):
                    request.object = userdata.products.objects[userdata.index]
                    return request

                @smach.cb_interface(outcomes=['succeeded', 'aborted'])
                def place_response_cb(userdata, response):
                    if response.success:
                        return 'succeeded'
                    else:
                        return 'aborted'

                smach.StateMachine.add('PLACE_PRODUCT',
                                        ServiceState('place_product',
                                                      Manipulate,
                                                      request_cb=place_request_cb,
                                                      response_cb=place_response_cb,
                                                      input_keys=['products', 'index']),
                                        transitions={'succeeded':'continue', 'aborted':'continue'})

                @smach.cb_interface(input_keys=['products', 'index'])
                def return_request_cb(userdata, request):
                    request.object = userdata.products.objects[userdata.index]
                    return request

                @smach.cb_interface(outcomes=['succeeded', 'aborted'])
                def return_response_cb(userdata, response):
                    if response.success:
                        return 'succeeded'
                    else:
                        return 'aborted'

                smach.StateMachine.add('RETURN_PRODUCT',
                                        ServiceState('return_product',
                                                      Manipulate,
                                                      request_cb=return_request_cb,
                                                      response_cb=return_response_cb,
                                                      input_keys=['products', 'index']),
                                        transitions={'succeeded':'continue', 'aborted':'aborted'})

            Iterator.set_contained_state('STOCK_PRODUCTS_STATE',
                                         stock_sm,
                                         loop_outcomes=['continue'])

        smach.StateMachine.add('STOCK_PRODUCTS',
                                stock_it,
                                transitions={'succeeded':'NAVIGATE_TO_SHELF_B', 'aborted':'aborted'})

        navigate_to_shelf_b_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

        with navigate_to_shelf_b_sm:
            navigate_to_shelf_b_sm.add('GO_TO_NEAR_SHELF_B',
                                        ServiceState('goto_near_shelf', GoToNearShelf, request=GoToNearShelfRequest(2)),
                                        transitions={'succeeded':'DETECT_SHELF_B'})

            navigate_to_shelf_b_sm.add('DETECT_SHELF_B',
                                        ServiceState('detect_shelf_action', DetectShelf, request=DetectShelfRequest(1)),
                                        transitions={'succeeded':'GO_TO_FACEUP_POSITION', 'aborted':'aborted'})

            navigate_to_shelf_a_sm.add('GO_TO_FACEUP_POSITION',
                                        ServiceState('goto_faceup_base_position', Trigger),
                                        transitions={'succeeded':'succeeded'})

        smach.StateMachine.add('NAVIGATE_TO_SHELF_B',
                                navigate_to_shelf_b_sm,
                                transitions={'succeeded':'DETECT_AND_RECOVER_SANDWICH'})

        smach.StateMachine.add('DETECT_AND_RECOVER_SANDWICH',
                                ServiceState('detect_and_recover_sandwich',
                                              DetectAndRecoverSandwich,
                                              response_slots=['normal_sandwiches']),
                                transitions={'succeeded':'FACEUP_SANDWICHES'})

        @smach.cb_interface(input_keys=['normal_sandwiches'])
        def faceup_request_cb(userdata, request):
            request.sandwiches = userdata.normal_sandwiches
            return request

        @smach.cb_interface(outcomes=['succeeded', 'aborted'])
        def faceup_response_cb(userdata, response):
            if response.success:
                return 'succeeded'
            else:
                return 'aborted'

        smach.StateMachine.add('FACEUP_SANDWICHES',
                                ServiceState('faceup_standing_sandwiches',
                                              FaceupSandwich,
                                              request_cb=faceup_request_cb,
                                              response_cb=faceup_response_cb,
                                              input_keys=['normal_sandwiches']
                                              ),
                                transitions={'succeeded':'GO_TO_HOME_POSITION'})


        smach.StateMachine.add('GO_TO_HOME_POSITION',
                                ServiceState('goto_home_position', Trigger),
                                transitions={'succeeded':'succeeded'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    rospy.sleep(3)

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
