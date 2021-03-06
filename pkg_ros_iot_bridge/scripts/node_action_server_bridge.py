#!/usr/bin/env python
'''declaration of shebang'''

# ROS Node - Action Server - IoT ROS Bridge

import threading
import rospy
import actionlib
import requests

from pkg_ros_iot_bridge.msg import msgRosIotAction    # Message Class used by ROS Actions
from pkg_ros_iot_bridge.msg import msgRosIotGoal      # Message Class used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult    # Message Class used for Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub         # Message Class for MQTT Subscription Messages

from pkg_task1.msg import msgTurtleResult             # Msg for the listening

from pyiot import iot                                 # Custom Python Module to perfrom MQTT Tasks


class IotRosBridgeActionClient:
    ''' A class to carry out the publishing to MQTT hive '''                           # Action Client CLass is defined here

    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_iot_ros',
                                          msgRosIotAction)				#lient is initialized

        # Dictionary to Store all the goal handels
        #self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_pyiot')				#Mqtt topic on which to publish is stored here
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
		
        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    # This function will be called when there is change of state in the Action Client State Machine
    def on_transition(self, goal_handle):
        '''from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.'''

        result = msgRosIotResult()							#Function is called when change of state is detected and index is assigned

        index = 1

    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        '''This function is used to send Goals to Action Server'''

        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol							#GOal is sent to the server so as to publish on the topic
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")

        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle


class IotRosBridgeActionServer:								#THe Action Server class initialised here

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
        * self.on_goal - It is the fuction pointer which points to a function which will be called
                         when the Action Server receives a Goal.

        * self.on_cancel - It is the fuction pointer which points to a function which will be called
                         when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_spread_id = param_config_iot['google_apps']['spread_sheet_id']	# Google spread sheet id is stored as parameter	
        print param_config_iot

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on(/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)

        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")


    def mqtt_sub_callback(self, client, userdata, message):					#Message to be published on Mqtt is sent by this function
        '''This is a callback function for MQTT Subscriptions'''

        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)


    def on_goal(self, goal_handle):
        ''' This function will be called when Action Server receives a Goal'''

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                # Start new thread to process new goal from the client
                # 'self.process_goal' -
                # is function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return


    def process_goal(self, goal_handle):
        '''This function is called is a separate thread to process Goal.'''

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()


        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success is True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    def on_cancel(self, goal_handle):
        ''' This function will be called when Goal Cancel request is send to the Action Server'''

        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        '''Create a Goal Message object'''

        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")

        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

def func_callback_topic_my_topic(obj_msg_result):
    ''' This function will act as callback'''
    action_client = IotRosBridgeActionClient()
    param_config_iot = rospy.get_param('config_pyiot')
    config_google_spread_id1 = param_config_iot['google_apps']['spread_sheet_id1']
    config_google_spread_id2 = param_config_iot['google_apps']['spread_sheet_id2']

    msg_result = msgTurtleResult()
    msg_result.final_x = obj_msg_result.final_x
    msg_result.final_y = obj_msg_result.final_y
    msg_result.final_theta = obj_msg_result.final_theta

    mqttval = action_client.send_goal("mqtt", "pub",
                                      action_client._config_mqtt_pub_topic,
                                      str(obj_msg_result))
    #We pass the function which will call iot module and in it, the google script function will print the values on respective sheets
    googlepass = iot.google_script_val(config_google_spread_id1,
                                                 msg_result,"Sheet1")
    googlepass1 = iot.google_script_val(config_google_spread_id2,
                                                 msg_result,"task1")

# Main
def main():
    ''' This is the main function'''

    # 1. Initialize ROS Node
    rospy.init_node('node_action_server_ros_iot_bridge')

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("msg_from_act_server", msgTurtleResult, func_callback_topic_my_topic)

    # 3. Create IOT ROS Bridge Action Server object.
    action_server = IotRosBridgeActionServer()

    # 4. Do not exit and loop forever.
    rospy.spin()


if __name__ == '__main__':
    main()
