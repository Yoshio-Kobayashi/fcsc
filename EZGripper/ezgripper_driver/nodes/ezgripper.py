#!/usr/bin/python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, SAKE Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##

#
#  If you want to modify this program, you can find the "Main program" and
#  "Main loop" by searching for these terms.  They exist near the end of this file.
#

import rospy
from std_srvs.srv import Empty, EmptyResponse
from libezgripper import create_connection, Gripper
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from functools import partial
from sensor_msgs.msg import JointState

# max_joint_value = 1.57
# min_joint_value = -0.27;
max_joint_value = 1.94
min_joint_value = 0.0;
joint_step = 100.0 / (max_joint_value - min_joint_value)

def calibrate_srv(gripper, msg):
    rospy.loginfo("Calibrate service: request received")
    gripper.calibrate()
    gripper.open()
    rospy.loginfo("Calibrate service: request completed")
    return EmptyResponse()

class GripperActionServer:
    def __init__(self, action_name, gripper):
        self.gripper = gripper
        self.action_server = actionlib.SimpleActionServer(action_name, GripperCommandAction, self.gripper_action_execute, False)
        self.action_server.start()

    def gripper_action_execute(self, goal):
        rospy.loginfo("Execute goal: position=%.1f, max_effort=%.1f"%
                      (goal.command.position, goal.command.max_effort))


        # joint_position = min_joint_value + joint_step * goal.command.position;
        # joint_position = (goal.command.position - min_joint_value) * joint_step
        joint_position = (max_joint_value - goal.command.position) * joint_step
        goal.command.position = joint_position


        if goal.command.max_effort == 0.0:
            rospy.loginfo("Release torque: start")
            self.gripper.release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            self.gripper.goto_position(goal.command.position, goal.command.max_effort)
            rospy.loginfo("Go to position: done")

        result = GripperCommandResult()
        result.position = goal.command.position #not necessarily the current position of the gripper if the gripper did not reach its goal position.
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = True
        self.action_server.set_succeeded(result)

def send_diags():
    # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
    msg = DiagnosticArray()
    msg.status = []
    msg.header.stamp = rospy.Time.now()

    for gripper in grippers:
        for servo in gripper.servos:
            status = DiagnosticStatus()
            status.name = "Gripper '%s' servo %d"%(gripper.name, servo.servo_id)
            status.hardware_id = '%s'%servo.servo_id
            temperature = servo.read_temperature()
            status.values.append(KeyValue('Temperature', str(temperature)))
            status.values.append(KeyValue('Voltage', str(servo.read_voltage())))

            if temperature >= 70:
                status.level = DiagnosticStatus.ERROR
                status.message = 'OVERHEATING'
            elif temperature >= 65:
                status.level = DiagnosticStatus.WARN
                status.message = 'HOT'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'

            msg.status.append(status)

    diagnostics_pub.publish(msg)


# Main Program

rospy.init_node('ezgripper')
rospy.loginfo("Started")

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
gripper_params = rospy.get_param('~grippers')

diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
joint_state_pub = rospy.Publisher('/ezgripper/joint_states', JointState, queue_size=10)

connection = create_connection(port_name, baud)

all_servos = []
references = []
grippers = []

for gripper_name, servo_ids in gripper_params.iteritems():
    gripper = Gripper(connection, gripper_name, servo_ids)
    all_servos += gripper.servos

    if gripper_name == 'both' :
        gripper.calibrate()

    gripper.open()

    references.append( rospy.Service('~'+gripper_name+'/calibrate', Empty, partial(calibrate_srv, gripper)) )
    references.append( GripperActionServer('~'+gripper_name, gripper) )

    grippers.append(gripper)

# Main Loop

r = rospy.Rate(20) # hz
diags_last_sent = 0
while not rospy.is_shutdown():

    # publish gripper joint state
    now = rospy.get_time()
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    for gripper in grippers:
        if gripper.name == 'top':
            joint_state_msg.name.append('top_ezgripper_knuckle_palm_L1_1')
        elif gripper.name == 'bottom':
            joint_state_msg.name.append('bottom_ezgripper_knuckle_palm_L1_1')
        else :
            continue
        grip_pos = gripper.get_position()
        grip_joint = max_joint_value - grip_pos / joint_step
        joint_state_msg.position.append(grip_joint)
    joint_state_pub.publish(joint_state_msg)

    if now - diags_last_sent > 1.0:
        try:
            send_diags()
            diags_last_sent = now
        except Exception, e:
            rospy.logerr("Exception while reading diagnostics: %s"%e)

    for servo in all_servos:
        try:
            servo.check_overload_and_recover()
        except Exception, e:
            rospy.logerr("Exception while checking overload: %s"%e)
            servo.flushAll()

    r.sleep()

rospy.loginfo("Exiting")
