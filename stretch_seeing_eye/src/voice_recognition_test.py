import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

CONFIDENCE_THRESHOLD = 0.95

def callback(msg: SpeechRecognitionCandidates):
    rospy.logdebug(msg)
    rospy.logdebug(msg.transcript)

    confidence = msg.confidence[0]
    transcript =  msg.transcript[0].split(' ')
    if transcript[0] == 'stop':
        rospy.logdebug('STOPPING')
        return
    if confidence > CONFIDENCE_THRESHOLD and len(transcript) >= 3:
        if transcript[0] == 'navigate' and transcript[1] == 'to':
            rospy.logdebug("Navigation message")
    else:
        rospy.logdebug('Confidence rating is low')

    
    return


if __name__ == '__main__':
    rospy.init_node('voice_recognition_test', log_level=rospy.DEBUG)

    sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, callback)

    while not rospy.is_shutdown():
        rospy.spin()