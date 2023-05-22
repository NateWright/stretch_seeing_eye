import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from stretch_seeing_eye.srv import Waypoint, WaypointRequest, WaypointResponse

CONFIDENCE_THRESHOLD = 0.95
service_client = None

def callback(msg: SpeechRecognitionCandidates):
    rospy.logdebug(msg)
    rospy.logdebug(msg.transcript)

    confidence = msg.confidence[0]
    transcript =  msg.transcript[0].split(' ')
    if transcript[0] == 'stop':
        rospy.logdebug('STOPPING')
        return
    if confidence > 0 and len(transcript) >= 3:
        if transcript[0] == 'navigate' and transcript[1] == 'to':
            rospy.logdebug("Navigation message")
            service_client.call(WaypointRequest(data=transcript[2]))
                
    else:
        rospy.logdebug('Confidence rating is low')

    
    return


if __name__ == '__main__':
    rospy.init_node('voice_recognition_test', log_level=rospy.DEBUG)

    service_client = rospy.ServiceProxy('/stretch_seeing_eye/navigate_to_waypoint', Waypoint)
    sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, callback)

    while not rospy.is_shutdown():
        rospy.spin()