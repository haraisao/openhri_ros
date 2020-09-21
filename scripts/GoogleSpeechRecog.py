#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Google Speech Recognition component

Copyright (C) 2017
  Isao Hara
  National Institute of Advanced Industrial Science and Technology (AIST),
  Japan
  All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''
from __future__ import print_function
import sys, os, platform
import time, struct, traceback, tempfile, optparse

import json
from pydub import AudioSegment
from pydub.silence import *

from xml.dom.minidom import Document

import openhri.utils as utils
from openhri.google_asr.config import config
from openhri import CloudSpeechRecogBase

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String


#
#
__doc__ = 'Google Speech Recognition component.'
__version__ = '1.0'

PYTHON_MAJOR_VERSION = sys.version_info.major
if PYTHON_MAJOR_VERSION == 2:
  from io import open
  from urllib import urlencode
else:
  from urllib.parse import urlencode

import urllib3 

#
#  
#
class GoogleSpeechRecogWrap(CloudSpeechRecogBase):
  #
  #  Constructor
  #
  def __init__(self, rtc, language='ja-JP'):
    CloudSpeechRecogBase.__init__(self, language)

    self._endpoint = 'http://www.google.com/speech-api/v2/recognize'
    self._http = urllib3.PoolManager()

    prop = rtc._manager._config
    if prop.getProperty("google.speech.apikey") :
      self._apikey = prop.getProperty("google.speech.apikey")

    if prop.getProperty("google.speech.lang") :
      self._lang=prop.getProperty("google.speech.lang")

    if prop.getProperty("google.speech.logdir") :
      self._logdir=prop.getProperty("google.speech.logdir")

    if prop.getProperty("google.speech.save_wav") :
      if prop.getProperty("google.speech.save_wav") == 'YES':
        self._logger = True


  #
  #  Set ApiKey
  #
  def set_apikey(self, key):
    self._apikey = key


  #
  #  Request Google Voice Recognition
  #
  def request_speech_recog(self, data):
    query_string = {'output': 'json', 'lang': self._lang, 'key': self._apikey}
    url = '{0}?{1}'.format(self._endpoint, urlencode(query_string)) 
    headers = {'Content-Type': 'audio/l16; rate=16000'}
    voice_data = bytearray(data)

    try:
      result = self._http.urlopen('POST', url, body=voice_data, headers=headers)
      if result.status == 200:
        response = result.data
        return response.decode('utf-8').split()
      else:
        rospy.logerr("Fail to ASR: status = %d", result.status)
        return ["Error"]

    except:
      print (url)
      print (traceback.format_exc())
      return ["Error"]

#
#  GoogleSpeechRecog Class
#
class GoogleSpeechRecog(object):
  #
  #  Constructor
  #
  def __init__(self, manager):
    self._recog = None
    self._copyrights=[]
    self._lang = [ "ja-JP" ]
    self._min_silence = [ 200 ]
    self._silence_thr = [ -20 ]
    self._min_buflen =  [ 8000 ]
    self._audio_topic = "/audio_capture/audio"

    self._manager=manager


  #
  #  OnInitialize
  #
  def onInitialize(self):
    rospy.loginfo("GoogleSpeechRecogRos version " + __version__)
    rospy.loginfo("Copyright (C) 2017 Isao Hara")

    #
    # Read parameters
    prop = self._manager._config
    if prop.getProperty("google.speech.min_silence") :
      self._min_silence = [ int(prop.getProperty("google.speech.min_silence")) ]
    if prop.getProperty("google.speech.silence_thr") :
      self._silence_thr = [ int(prop.getProperty("google.speech.silence_thr")) ]
    if prop.getProperty("google.speech.min_buflen") :
      self._min_buflen = [ int(prop.getProperty("google.speech.min_buflen")) ]
    if prop.getProperty("google.speech.audio_topic") :
      self._audio_topic = prop.getProperty("google.speech.audio_topic")

    #
    # create inport for audio stream
    self._audio_sub = rospy.Subscriber(self._audio_topic, AudioData, self.onData)

    #
    # create outport for result
    self._asr_result = rospy.Publisher('/asr/result', String, queue_size=10)

    rospy.loginfo("This component depends on following softwares and datas:")
    rospy.loginfo('')
    for c in self._copyrights:
      for l in c.strip('\n').split('\n'):
        rospy.loginfo('  '+l)
      rospy.loginfo('')

    return True

  #
  #  OnFinalize
  #
  def onFinalize(self):
    if self._recog:
      self._recog.terminate()
    return True

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    self._recog = GoogleSpeechRecogWrap(self, self._lang[0])
    self._recog.setcallback(self.onResult)

    self._recog.set_voice_detect_param(int(self._min_silence[0]),  int(self._silence_thr[0]), int(self._min_buflen[0]))

    if self._recog._apikey:
      self._recog.start()
      return True
    else:
      return False

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id=0):
    self._recog.terminate()
    return True

  #
  #  OnData (Callback from Subscriber)
  #
  def onData(self, data):
    if self._recog:
      self._recog.write(data.data)
    return

  #
  #  OnExecute (Do nothing)
  #
  def onExecute(self, ec_id=0):
    return RTC.RTC_OK

  #
  #  OnResult
  #
  def onResult(self, data):
    doc = Document()
    listentext = doc.createElement("listenText")
    doc.appendChild(listentext)

    if len(data) <= 1:
      listentext.setAttribute("state","RecognitionFailed")
    else:
      try:
        data.pop(0)
        res = ''.join(data)
        #print res;
        result=json.loads(res)
        i=0
        for r in result['result'][0]['alternative']:
          i += 1
          rank = str(i)
          if 'confidence' in r :
            score=str(r['confidence'])
          else:
            score=0.0
          text=r['transcript']
          hypo = doc.createElement("data")
          hypo.setAttribute("rank", rank)
          hypo.setAttribute("score", score)
          hypo.setAttribute("likelihood", score)
          hypo.setAttribute("text", text)
          rospy.loginfo("#%s: %s (%s)" % (rank, text.encode('utf-8'), score))
          listentext.appendChild(hypo)

        listentext.setAttribute("state","Success")

      except:
        print (traceback.format_exc())
        listentext.setAttribute("state","ParseError")

    res_data = doc.toxml(encoding="utf-8")
    self._asr_result.publish(res_data)
    return

  #
  #
  def shutdown(self):
    self._recog.terminate()
    return
#
#  Manager Class
#
class GoogleSpeechRecogManager:
  #
  #  Constructor
  #
  def __init__(self):
    parser = utils.MyParser(version=__version__, description=__doc__)
    utils.addmanageropts(parser)

    try:
      opts, args = parser.parse_args()
    except optparse.OptionError as e:
      rospy.logerr( 'OptionError:' + str(e))
      sys.exit(1)

    if opts.configfile is None:
      try:
        cfgname = os.environ['OPENHRI_ROOT'] + "/etc/google_speech.conf".replace('/', os.path.sep)
        if os.path.exists(cfgname):
          opts.configfile = cfgname
      except:
        pass

    self.name = 'google_speech_recog'
    self._comp = None
    self._config = config(configfile=opts.configfile)

    rospy.init_node(self.name, anonymous=True)
    self.moduleInit()

  #
  #  Start component
  #
  def start(self):
    self._comp.onActivated()
    rospy.spin()
    return

  def shutdown(self):
    self._comp.shutdown()
    return
  #
  #
  def create_component(self, name):
    cls = globals()[name]
    comp=cls(self)
    comp.onInitialize()
    return comp
  #
  #  Initialize rtc
  #
  def moduleInit(self):
    self._comp = self.create_component("GoogleSpeechRecog")
    return

#
#
g_manager = None

#
#
def main():
  global g_manager
  g_manager = GoogleSpeechRecogManager()
  g_manager.start()
  g_manager.shutdown()


#
#  Main
#
if __name__=='__main__':
  main()
