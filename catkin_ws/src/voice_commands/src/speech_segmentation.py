#!/usr/bin/env python

# this node processes the vocal input the user provided through speech_to_text.py using the Speech Message

# NLP librariesinstallation
    # pip install -U pip setuptools wheel
    # pip install spacy
    # python -m spacy download en_core_web_sm

import rospy
from std_msgs.msg import String
from voice_commands.msg import Speech, Command
import numpy as np

import re
import nltk
import spacy
from spacy.lang.en.stop_words import STOP_WORDS
from nltk.tree import *
from nltk.tokenize import word_tokenize

message = Speech()
command = Command()
processed = False

def process(msg):
    # process the message
    words = word_tokenize(msg)
    pos_tags = nltk.pos_tag(words)
    verb = []; object = []; adjective = []
    for i in range(len(pos_tags)):
        word, tag = pos_tags[i]
        if tag.startswith('VB'):
            verb.append(word)
        elif re.match("NN.*", tag):
            object.append(word)
            if word == 'place':
                verb.append(word)
                object.remove(word)
            if word == 'blue':
                adjective.append(word)
                object.remove(word)
        elif re.match("JJ.*", tag):
            adjective.append(word)

    # invalid: locate, relocate, give, hold, grab, bring, fetch
    word1 = 'pick' # grasp, lift, collect, take
    word2 = 'place' # put, set, position, deposit
    colors = ['red', 'green', 'blue', 'yellow']

    # find the corresponding command
    if verb != []:
        if verb.__contains__('place'):
            command.verb = 'place'
            if adjective == []:
                command.color = 'previous'
                command.valid = True
            else:
                if colors.__contains__(adjective[0]):
                    command.color = adjective[0]
                    command.valid = True
                else:
                    command.color = '0'
                    command.verb = '0'
                    command.valid = False
        elif verb.__contains__('pick'):
            command.verb = 'pick'
            if adjective == []:
            # cannot pick an object of unknown color -> invalid command
                command.verb = '0'
                command.color = '0'
                command.valid = False
            else:
                if colors.__contains__(adjective[0]):
                    command.color = adjective[0]
                    command.valid = True
                else:
                    command.color = '0'
                    command.verb = '0'
                    command.valid = False
        else:
            command.color = '0'
            command.verb = '0'
            command.valid = False
    else:
        # command without verb is invalid
        command.verb = ''
        command.color = ''
        command.valid = False
    return command.verb, command.color, command.valid 


def result_segmentation(msg):
    message.text = msg.text
    message.stop = msg.stop

    if message.stop == True:
        print(' User command: Stopping')
    else:
        print(' User command: ', message.text)
        [command.verb, command.color, command.valid] = process(message.text)
        global processed
        processed = True

def speech_segmentation():
    rospy.init_node('speech_segmentation', anonymous=True) 
    pub = rospy.Publisher('/command', Command, queue_size=10)   
    rospy.Subscriber('/stt',Speech, result_segmentation)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if command.valid:
            print(' Sending the command: ', command.verb, " ", command.color)
            pub.publish(command)
            command.valid = False
        else: 
            if (command.verb == '0') or (command.color == '0'):
                print(' Invalid command')
                break
            else:
                pass
        #rate.sleep()

    
if __name__ == '__main__':
    try:
        speech_segmentation()
    except rospy.ROSInterruptException:
        pass