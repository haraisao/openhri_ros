#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''OpenJTalk speech synthesis component

Copyright (C) 2010
    Yosuke Matsusaka
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.

Copyright (C) 2017
    Isao Hara,
    National Institute of Advanced Industrial Science and Technology (AIST), Japan
    All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import os
import sys
import time
import subprocess
import signal
import traceback
import platform
import optparse
import re

from openhri.openjtalk.config import config
import openhri
from openhri.utils import getHriDir,native_path

import rospy


__doc__ = 'Japanese speech synthesis component.'
__version__ = openhri.__version__

'''
NAIST Japanese Dictionary
Version 0.6.1-20090630 (http://naist-jdic.sourceforge.jp/)
Copyright (C) 2009 Nara Institute of Science and Technology
All rights reserved.

open_jtalk - The Japanese TTS system "Open JTalk"

  usage:
       open_jtalk [ options ] [ infile ]
  options:                                                    [ def][ min-- max]
    -x  dir      : dictionary directory                       [ N/A]
    -m  htsvoice : HTS voice files                            [ N/A]
    -ow s        : filename of output wav audio (generated speech) [ N/A]
    -ot s        : filename of output trace information       [ N/A]
    -s  i        : sampling frequency                         [auto][   1--    ]
    -p  i        : frame period (point)                       [auto][   1--    ]
    -a  f        : all-pass constant                          [auto][ 0.0-- 1.0]
    -b  f        : postfiltering coefficient                  [ 0.0][ 0.0-- 1.0]
    -r  f        : speech speed rate                          [ 1.0][ 0.0--    ]
    -fm f        : additional half-tone                       [ 0.0][    --    ]
    -u  f        : voiced/unvoiced threshold                  [ 0.5][ 0.0-- 1.0]
    -jm f        : weight of GV for spectrum                  [ 1.0][ 0.0--    ]
    -jf f        : weight of GV for log F0                    [ 1.0][ 0.0--    ]
    -g  f        : volume (dB)                                [ 0.0][    --    ]
    -z  i        : audio buffer size (if i==0, turn off)      [   0][   0--    ]
  infile:
    text file                                                 [stdin]
'''

#### for python2
from io import open

#
#  Read file
#
def read_file_contents(fname, encoding='utf-8'):
  try:
    f=open(fname,'r', encoding=encoding)
    contents = f.read()
    f.close()
    return contents
  except:
    return ""

#
#  OpenJTalk Process Wrapper
#
class OpenJTalkWrap(openhri.VoiceSynthBase):
  #
  #
  #
  def __init__(self, prop):
    openhri.VoiceSynthBase.__init__(self)
    self._conf = config()
    self._args = ()

    self._sampling_rate = 0
    self._frame_period = 0
    self._all_pass = -1
    self._postfiltering_coefficent = 0.0
    self._speed_rate = 1.0
    self._addtional_half_tone = 0.0
    self._threshold = 0.5
    self._gv_spectrum = 1.0
    self._gv_log_f0 = 1.0
    self._volume = 0.0
    self._proc = None

    self._basedir = getHriDir()

    self._config = prop
    top_dir = self.bindParameter("openjtalk.top_dir", None, "")
    if top_dir :
      top_dir = re.sub('^%d0', self._basedir[:2], top_dir)
      self._conf.openjtalk_top(native_path(top_dir))

    sox_dir = self.bindParameter("openjtalk.sox_dir", None, "")
    if sox_dir :
      sox_dir = re.sub('^%d0', self._basedir[:2], sox_dir)
      self._conf.sox_top(native_path(sox_dir))

    male_dir = self.bindParameter("openjtalk.phonemodel_male_ja", None, "")
    if male_dir :
      self._conf._openjtalk_phonemodel_male_ja=make_dir

    female_dir = self.bindParameter("openjtalk.phonemodel_female_ja", None, "")
    if female_dir :
      self._conf._openjtalk_phonemodel_female_ja=female_dir

    #
    # get copyright infomationn
    self._copyrights = []
    cmdarg = [ self._conf._openjtalk_bin ]
    self._proc = subprocess.Popen(cmdarg, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
      stdoutstr, stderrstr = self._proc.communicate()
      for l in stderrstr.decode('utf-8').replace('\r', '').split('\n\n'):
        if l.count('All rights reserved.') > 0:
          self._copyrights.append(l)
    except:
      print("Error in self._proc.communicate()")


    ##################################
    #  read copyright
    self._copyrights.append(read_file_contents('hts_voice_copyright.txt'))
    self._copyrights.append(read_file_contents('mmdagent_mei_copyright.txt'))

  #
  #  TTS conversion
  #
  def synthreal(self, data, samplerate, character):
    textfile = self.gettempname()
    wavfile  = self.gettempname()
    logfile  = self.gettempname()

    # text file which specifies synthesized string
    fp = open(textfile, 'w', encoding='utf-8')
    fp.write(u"%s\n" % (data,))
    fp.close()

    # command line for OpenJTalk 
    cmdarg = [ self._conf._openjtalk_bin ]
    #
    #  select phonemodel
    if character == "female":
      cmdarg.extend(["-m", self._conf._openjtalk_phonemodel_female_ja])
    else:
      cmdarg.extend(["-m", self._conf._openjtalk_phonemodel_male_ja])
    #
    #  audio buffer size
    #cmdarg.extend(["-z", "2000"])
    #
    #  sampling rate
    if self._sampling_rate > 0:
      cmdarg.extend(["-s", str(self._sampling_rate)])

    #   all-pass constant 
    if self._all_pass >= 0.0 and self._all_pass <= 1.0:
      cmdarg.extend(["-a", str(self._all_pass)])

    #   postfiltering coefficient  
    if self._postfiltering_coefficent >= 0.0 and self._postfiltering_coefficent <= 1.0:
      cmdarg.extend(["-b", str(self._postfiltering_coefficent)])

    #
    #
    #  speech speed rate
    if self._speed_rate > 0:
      cmdarg.extend(["-r", str(self._speed_rate)])

    #  additional half-tone
    cmdarg.extend(["-fm", str(self._addtional_half_tone)])

    #   voiced/unvoiced threshold
    if self._threshold >= 0.0 and self._threshold <= 1.0:
      cmdarg.extend(["-u", str(self._threshold)])

    #  weight of GV for spectrum
    if self._gv_spectrum >= 0.0:
      cmdarg.extend(["-jm", str(self._gv_spectrum)])

    #  weight of GV for log F0
    if self._gv_log_f0 >= 0.0:
      cmdarg.extend(["-jf", str(self._gv_log_f0)])

    #  volume (dB) 
    if self._conf._platform == "Windows" :
      cmdarg.extend(["-g", str(self._volume)])


    #
    # dictionary directory
    cmdarg.extend(["-x", self._conf._openjtalk_dicfile_ja])
    #
    # filename of output wav audio and filename of output trace information
    cmdarg.extend(["-ow", wavfile, "-ot", logfile])
    #
    # text file(input)
    cmdarg.append(textfile)

    #print (' '.join(cmdarg))
    # run OpenJTalk
    #    String ---> Wav data
    p = subprocess.Popen(cmdarg)
    p.wait()

    # convert samplerate
    # normally openjtalk outputs 48000Hz sound.
    wavfile2 = self.gettempname()
    cmdarg = [self._conf._sox_bin, "-t", "wav", wavfile, "-r", str(samplerate), "-t", "wav", wavfile2]
    p = subprocess.Popen(cmdarg)
    p.wait()

    os.remove(wavfile)
    wavfile = wavfile2

    # read duration data
    d = openhri.parseopenjtalk()
    d.parse(logfile)
    durationdata = d.toseg()
    os.remove(textfile)
    os.remove(logfile)

    return (durationdata, wavfile)

  #
  #  set cachesize
  #
  def set_cachesize(self, n):
    self._cachesize = n

  #
  #  set params
  #
  def set_params(self, rtc):
    self._sampling_rate = int(rtc._sampling_rate[0])
    self._all_pass = float(rtc._all_pass[0])
    self._postfiltering_coefficent = float(rtc._postfiltering_coefficent[0])
    self._speed_rate = float(rtc._speed_rate[0])
    self._addtional_half_tone = float(rtc._addtional_half_tone[0])
    self._threshold = float(rtc._threshold[0])
    self._gv_spectrum = float(rtc._gv_spectrum[0])
    self._gv_log_f0 = float(rtc._gv_log_f0[0])
    self._volume = float(rtc._volume[0])
  #
  #  terminated
  #
  def terminate(self):
    if self._proc :
      self._proc.terminate()
    pass

#
#  OpenJTalkRos class
#
class OpenJTalkRos(openhri.VoiceSynthComponentBase):
  #
  # Constructor
  #
  def __init__(self, manager):
    openhri.VoiceSynthComponentBase.__init__(self, manager)
    self._cachesize=[1]
    self._sampling_rate=[0]
    self._all_pass = [-1.0]
    self._postfiltering_coefficent = [0.0]
    self._speed_rate=[1.0]
    self._addtional_half_tone = [0.0,]
    self._threshold = [0.5,]
    self._gv_spectrum = [1.0,]
    self._gv_log_f0 = [1.0,]
    self._volume = [0.0]

  #
  #  OnInitialize
  #
  def onInitialize(self):
    openhri.VoiceSynthComponentBase.onInitialize(self)

    self.bindParameter("cachesize", self._cachesize, "1", int)
    self.bindParameter("sampling_rate", self._sampling_rate, "0", int)
    self.bindParameter("all_pass", self._all_pass, "-1.0", float)
    self.bindParameter("postfiltering_coefficent", self._postfiltering_coefficent, "0.0",float)

    self.bindParameter("speed_rate", self._speed_rate, "1.0",float)
    self.bindParameter("half_tone", self._addtional_half_tone, "0.0",float)
    self.bindParameter("voice_unvoice_threshold", self._threshold, "0.5",float)
    self.bindParameter("gv_spectrum", self._gv_spectrum, "1.0",float)
    self.bindParameter("gv_log_f0", self._gv_log_f0, "1.0",float)
    self.bindParameter("volume", self._volume, "0.0",float)


    self._wrap = OpenJTalkWrap(self._config)
    self._copyrights = self._wrap._copyrights
    self.show_copyrights()

    return True
  #
  #
  #
  def onActivated(self, ec_id=0):
    self._wrap.set_params(self)
    self._wrap.set_cachesize(int(self._cachesize[0]))

    openhri.VoiceSynthComponentBase.onActivated(self, ec_id)
    return True

  #
  #  OnData (callback function)
  #
  def onData(self, data):
    self._wrap.set_params(self)

    openhri.VoiceSynthComponentBase.onData(self, data)
    return True

  def onShutdown(self):
    return

#
#  OpenJTalkRTC Manager class
#
class OpenJTalkRosManager(openhri.OpenHRI_Manager):
  #
  # Constructor
  #
  def __init__(self):
    openhri.set_manager_info(__version__, "%prog", __doc__)
    openhri.OpenHRI_Manager.__init__(self, name='OpenJtalk',
                                     conf_name='openjtalk.conf')
    self._config = config(config_file=self._opts.configfile)

    self._rate=10

    #
    #  Module Initializer
    #
  def moduleInit(self):
    self._comp['OpenJtalk'] = self.create_component(OpenJTalkRos)
    return


#
#  Main Function
#
if __name__=='__main__':
    manager = OpenJTalkRosManager()
    manager.init_node()
    manager.start()
    manager.shutdown()
