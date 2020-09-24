#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Google Text to Speech component

Copyright (C) 2019
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

from __future__ import print_function
import sys, os, platform
import time, struct, optparse
import traceback

import json, base64

import certifi
import urllib3

from pydub import AudioSegment
from pydub.silence import *

import openhri.utils as utils
import openhri.google_tts.config as config
import openhri
from openhri import __version__

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String


__doc__ = 'Google Text-to-Speech component.'

_EffectsProfile=('wearable', 'handset', 'headphone', 'small-bluetooth-speaker', 'medium-bluetooth-speaker', 'large-home-entertainment', 'large-automotive', 'telephony')

#
#  
#
class GoogleTextToSpeechWrap(openhri.ros_object):
  #
  #  Constructor
  #
  def __init__(self, node, language='ja-JP'):
    self._endpoint = "https://texttospeech.googleapis.com/v1/text:synthesize"
    self._http = urllib3.PoolManager(cert_reqs='CERT_REQUIRED',
                                     ca_certs=certifi.where())

    self._config = node._manager._config
    param=[""]
    self._apikey = self.bindParameter("google.tts.apikey",param, "")
    self._lang = self.bindParameter("google.tts.lang",param, "ja-JP")
    self._speekingRate = self.bindParameter("google.tts.speekingRate",param, "1.0", float)
    self._ssmlGender = self.bindParameter("google.tts.ssmlGender",param, "MALE")
    self._voiceName = self.bindParameter("google.tts.voiceName",param, "ja-JP-Wavenet-D")
    self._pitch = self.bindParameter("google.tts.pitch",param, "1.0", float)
    self._volumeGain = self.bindParameter("google.tts.volumeGain",param, "0", float)
    self._sampleRate = self.bindParameter("google.tts.sampleRate",param, "16000", int)
    self._effectsProfileId = self.bindParameter("google.tts.effectsProfieId",param, None)

    if not self._apikey :
      try:
        self._apikey = openhri.get_apikey(os.environ['HOME']+'/.openhri/google_apikey.txt')
      except:
        pass

  #
  #  Set ApiKey
  #
  def set_apikey(self, key):
    self._apikey = key

  #
  #
  def text2speech(self, text):
    url = self._endpoint+"?key="+self._apikey
    headers = {  'Content-Type' : 'application/json; charset=utf-8' }

    data = { "input": { "text" : text }, 
             "voice" : { 'languageCode' : self._lang    # en-US, ja-JP, fr-FR
                       , 'name' : self._voiceName
                       , 'ssmlGender' : self._ssmlGender # MALE, FEMALE, NEUTRAL
                       },
                       'audioConfig': { 
                          'audioEncoding':'LINEAR16'  # LINEAR16, MP3, OGG_OPUS
                       , 'speakingRate' : self._speekingRate   # [0.25: 4.0]
                       , 'pitch' : self._pitch         # [ -20.0: 20.0]
                       , 'volumeGainDb' : self._volumeGain 
                       , 'sampleRateHertz' : self._sampleRate # default is 22050
                      }
            }

    if self._effectsProfileId in _EffectsProfile:
      if self._effectsProfileId == 'telephony':
        data['audioConfig']['effectsProfileId'] = 'telephony-class-application'
      else:
        data['audioConfig']['effectsProfileId'] = self._effectsProfileId + "-class-device"


    #rospy.loginfo(data)
    try:
      result = self._http.urlopen('POST',url, body=json.dumps(data).encode(), headers=headers)
      response = result.data
      return response
    except:
      print ('Error')
      traceback.print_exc()
      return ""

#
#  GoogleTextToSpeechRos Class
#
class GoogleTextToSpeechRos(openhri.OpenHRI_Component):
  #
  #  Constructor
  #
  def __init__(self, manager):
    openhri.OpenHRI_Component.__init__(self, manager)
    self._tts = None
    self._lang = [ "ja-JP" ]


  #
  #  OnInitialize
  #
  def onInitialize(self):
    rospy.loginfo("GoogleTextToSpeechRos version " + __version__)
    rospy.loginfo("Copyright (C) 2019 Isao Hara")

    #
    # create inport for audio stream
    self._inport = rospy.Subscriber('/tts/data', String, self.onData)

    #
    # create outport for result
    self._outdata = AudioData()
    self._outport = rospy.Publisher('/audio_play/audio', AudioData, queue_size=10)

    #
    #
    self.show_copyrights()
    return True

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    self._tts = GoogleTextToSpeechWrap(self, self._lang[0])

    if self._tts._apikey:
      return True
    else:
      print("=== No API KEY ===")
      return False

  #
  #  OnData (Callback from DataListener)
  #
  def onData(self, data):
    if self._tts:
      try:
        txt=data.data.encode('raw-unicode-escape').decode()
      except:
        txt=data.data
      response = self._tts.text2speech(txt)
        
      if response :
        result=json.loads(response.decode())
        try:
          audio = base64.b64decode(result['audioContent'].encode())

          if len(audio) > 44:
            self._outdata.data = audio[44:]
            self._outport.publish(self._outdata)
        except:
            rospy.logerror("Error : no audia data")
    return


#
#  Manager Class
#
class GoogleTextToSpeechManager(openhri.OpenHRI_Manager):
  #
  #  Constructor
  #
  def __init__(self):
    openhri.set_manager_info(__version__, "%prog", __doc__)

    openhri.OpenHRI_Manager.__init__(self, name='google_tts',
                                     conf_name='google_tts.conf')

    self._config = config(config_file=self._opts.configfile)

  #
  #  Initialize node
  #
  def moduleInit(self):
    self._comp['GoogleTTS'] = self.create_component(GoogleTextToSpeechRos)
    return

#
#
g_manager = None

#
#
def main():
  g_manager = GoogleTextToSpeechManager()
  g_manager.init_node()
  g_manager.start()
  g_manager.shutdown()


#
#  Main
#
if __name__=='__main__':
  main()
