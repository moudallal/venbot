#!/usr/bin/env python

# need to install SpeechRecognition - pip install SpeechRecognition
# run by calling - rosrun ros_vosk speech_rec

import rospy
from std_msgs.msg import String, Bool
from voice_commands.msg import Speech
import speech_recognition as sr

def speech_rec():
    rospy.init_node('speech_to_text', disable_signals=True) 
    pub = rospy.Publisher('/stt',Speech, queue_size=10)
    rate = rospy.Rate(1)

    msg = Speech()
    r = sr.Recognizer()
    s = sr.Microphone()
    with s as source:
        
        while not rospy.is_shutdown():
            try:
                print("Start speaking")
                audio = r.listen(source)
                txt = r.recognize_google(audio)
                print("You said: " + txt)
                if txt.__contains__('stop'):
                    msg.stop = True
                    msg.text = txt
                    rospy.signal_shutdown('Stopping')
                else:
                    msg.text = txt
                    msg.stop = False
                pub.publish(msg)
            except sr.UnknownValueError:
                print('Try again')
        #rate.sleep()


if __name__ == '__main__':
    try:
        speech_rec()
    except (KeyboardInterrupt) as e:
        pass