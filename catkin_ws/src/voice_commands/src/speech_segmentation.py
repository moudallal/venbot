#!/usr/bin/env python3

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

import requests
API_KEY = "c46040cd-8c0a-4f8a-97da-46a31fa35a25"
nlp = spacy.load('en_core_web_sm')

message = Speech()
command = Command()
command.valid = False

def process(msg):
    # process the message
    doc = nlp(msg)
    verb = []; adjective = []
    for token in doc:
        if token.pos_ == 'VERB':
            verb.append(token.text)
        elif token.pos_ == 'ADJ':
            adjective.append(token.text)
        if token.text == 'place':
            verb.append('place')
        if token.text == 'sort':
            verb.append('sort')

    # invalid for this API: locate, relocate, give, hold, grab, bring, fetch
    word1 = 'pick' # grasp, lift, collect, take
    word2 = 'place' # put, set, position, deposit
    colors = ['red', 'green', 'blue', 'yellow']
   
    if verb != []:
        # if command contains verb, check if verb is a synonym of pick/place
        verb_test = verb[0]
        url_pick = f"https://www.dictionaryapi.com/api/v3/references/thesaurus/json/{word1}"
        url_place = f"https://www.dictionaryapi.com/api/v3/references/thesaurus/json/{word2}"
        params = {'key': API_KEY}
        response_pick = requests.get(url_pick, params=params)
        response_place =  requests.get(url_place, params=params)
        results_pick = response_pick.json()
        results_place = response_place.json()
        synonyms_pick = set()
        synonyms_place = set()
        for result in results_place:
            if "meta" in result and "syns" in result["meta"]:
                for synset in result["meta"]["syns"]:
                    synonyms_place.update(synset)
        for result in results_pick:
            if "meta" in result and "syns" in result["meta"]:
                for synset in result["meta"]["syns"]:
                    synonyms_pick.update(synset)
        
        if verb_test == 'sort':
            pass
        elif verb_test in synonyms_pick:
            verb[0] = 'pick'
        elif verb_test in synonyms_place:
            verb[0] = 'place'
        else:
            verb[0] = 'invalid'            
        
        '''
        # alternative to API - dictionary
        pick_synonyms = ['pick','hold','take','collect','grasp','lift','grab','bring','fetch','get']
        place_synonyms = ['place','put','set','locate','relocate','position','deposit']
        if verb[0] in pick_synonyms:
            verb[0] = 'pick'
        elif verb[0] in place_synonyms:
            verb[0] = 'place'
        else:
            verb[0] = '0'
        '''
            
        # find the corresponding command
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
        elif verb.__contains__('sort'):
            command.verb = 'sort'
            command.color = 'all'
            command.valid = True
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
                command.valid = True
                pass
            else:
                pass
            processed = False
        #rate.sleep()

   
if __name__ == '__main__':
    try:
        speech_segmentation()
    except rospy.ROSInterruptException:
        pass