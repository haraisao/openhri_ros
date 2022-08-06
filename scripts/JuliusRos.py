#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''Julius speech recognition component

Copyright (C) 2010
    Yosuke Matsusaka
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.

Copyright (C) 2017
    Isao Hara
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.

Copyright (C) 2020
    Isao Hara
    All rights reserved.

Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys, os, socket, subprocess, threading, platform
import time, struct, traceback, getopt, wave, tempfile
import traceback
import optparse
from glob import glob
from lxml import *
from bs4 import BeautifulSoup
from xml.dom.minidom import Document

import openhri
from openhri.julius.parsesrgs import *
from openhri.julius.config import config

import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData


__doc__ = "Julius (English and Japanese) speech recognition component."

'''
Dictation-kit
main.jconf:
  -d model/lang_m/bccwj.60k.bingram  # 単語2-gram,3-gramファイル(バイナリ形式）
  -v model/lang_m/bccwj.60k.htkdic   # 単語辞書ファイル
  -b 1500                            # 第1パスのビーム幅（ノード数
  -b2 100                            # 第2パスの仮説数ビームの幅（仮説数）
  -s 500                             # 第2パスの最大スタック数 (仮説数)
  -m 10000                           # 第2パスの仮説オーバフローのしきい値
  -n 30                              # 第2パスで見つける文の数（文数）
  -output 1                          # 第2パスで見つかった文のうち出力する数 （文数）
  -zmeanframe                        # フレーム単位のDC成分除去を行う (HTKと同処理)
  -rejectshort 800                   # 指定ミリ秒以下の長さの入力を棄却する
  
am-gmm.jconf
  -h model/phone_m/jnas-tri-3k16-gid.binhmm    # 音響HMM定義ファイル
  -hlist model/phone_m/logicalTri-3k16-gid.bin # 論理的に出現しうる triphone -> 定義されている triphoneの対応を指定した「HMMListファイル」
  -lmp  10 0  # 言語重みと挿入ペナルティ: 第1パス(2-gram)
  -lmp2 10 0  # 言語重みと挿入ペナルティ: 第2パス(3-gram)
  
------------------------------
ENVR-v5.4.Gmm.Bin:
-input mic
-htkconf wav_config
-h ENVR-v5.3.am
-hlist ENVR-v5.3.phn
-d ENVR-v5.3.lm
-v ENVR-v5.3.dct
-b 4000 
-b2 360 
-s 2000 
-m 8000 
-n 40 
-lmp 12 -6
-lmp2 12 -6
-fallback1pass
-multipath
-iwsp
-iwcd1 max
-spmodel sp
-no_ccd
-sepnum 150
-lookuprange 5 
-sb 80
-forcedict

'''

__version__ = '1.0'
PYTHON_MAJOR_VERSION = sys.version_info.major
if PYTHON_MAJOR_VERSION == 2:
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
#
#  Selector dialog for multi-files reading
#
def askopenfilenames(title=''):
  try:
    import Tkinter as tkinter
    import tkFileDialog as filedialog
  except:
    import tkinter
    import tkinter.filedialog as filedialog

  root = tkinter.Tk()
  root.withdraw()
  fTyp = [("","*")]
  fname = filedialog.askopenfilenames(filetypes = fTyp, initialdir = "", title=title)
  return fname


#  Julius Wrappper
#
class JuliusWrap(threading.Thread):
  CB_DOCUMENT = 1
  CB_LOGWAVE = 2
    
  #
  #  Constructor
  #
  def __init__(self, language='jp', node=''):
    threading.Thread.__init__(self)
    self._config = config()
    self._running = False
    self._platform = platform.system()
    self._gotinput = False
    self._lang = language
    self._memsize = "large"
    #self._memsize = "medium"

    self._logdir = tempfile.mkdtemp()
    self._callbacks = []
    self._grammars = {}
    self._firstgrammar = True
    self._activegrammars = {}
    self._prevdata = ''

    self._jconf_file = ""

    self._mode = 'grammar'
    #self._jcode = 'euc_jp'
    self._jcode = 'utf-8'
    self._p = None

    self._modulesocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._audiosocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._audiohost = "localhost"
    self._audioport = 0
    self._modulehost = "localhost"
    self._moduleport = 0

    if node :
      self._mode = node._mode
      prop = node._manager._config
      if prop.getProperty('julius.base_dir'):
        self._config = config(prop.getProperty('julius.base_dir'))

      if os.path.isfile(node._jconf_file):
        self._jconf_file = node._jconf_file

    ###########################################################
    if self._mode != "client" :
      if self.setupSubprocess():
        #print ("command line: %s" % " ".join(self._cmdline))
        #print (self._cmdline)
        self._p = subprocess.Popen(self._cmdline)
        self._running = True

    #####################################################
    #
    #   Connect to Julius (try ten times)
    time.sleep(1)
    rospy.loginfo ("connecting to ports")
    for retry in range(0, 10):
      try:
        self._modulesocket.connect((self._modulehost, self._moduleport))
      except socket.error:
        time.sleep(1)
        continue
      break

    for retry in range(0, 10):
      try:
        self._audiosocket.connect(( self._audiohost, self._audioport))
      except socket.error:
        time.sleep(1)
        continue
      break

    #
    # for grammar mode
    if self._mode != 'dictation' :
      self._modulesocket.sendall("INPUTONCHANGE TERMINATE\n".encode('utf-8'))

    rospy.loginfo ("JuliusWrap started")

  #
  # Parameter seting for Julius
  #
  def setupJuliusServer(self, host, mport, aport):
    self._modulehost = host
    self._moduleport = mport
    self._audiohost  = host
    self._audioport  = sport

  #
  # Parameter seting for Julius
  #
  def setupSubprocess(self):
    self._cmdline = []
    self._cmdline.append(self._config._julius_bin)

    if self._mode == 'dictation' :
      if self._lang in ('ja', 'jp'):
        # dictation-kit-v4.4(GMM版デフォルトパラメータ）ただし、outputを5に変更
        self._cmdline.extend(['-d', self._config._julius_bingram_ja])
        self._cmdline.extend(['-v', self._config._julius_htkdic_ja])
        self._cmdline.extend(['-h', self._config._julius_hmm_ja])
        self._cmdline.extend(['-hlist', self._config._julius_hlist_ja])
        self._cmdline.extend(["-b", "1500", "-b2", "100",
                                     "-s", "500" ,"-m", "10000"])
        self._cmdline.extend(["-n", "30", "-output", "5", "-zmeanframe",
                                  "-rejectshort" ,"800", "-lmp", '10' ,'0',
                                  '-lmp2', '10', '0'])

      elif self._config._julius_bin_en:
        self._cmdline = [self._config._julius_bin_en]
        self._cmdline.extend(['-d', self._config._julius_dict_ngram_en])
        self._cmdline.extend(['-v', self._config._julius_dict_dict_en])
        self._cmdline.extend(['-h', self._config._julius_dict_hmm_en])
        self._cmdline.extend(['-hlist', self._config._julius_dict_hlist_en])
        self._cmdline.extend(['-htkconf', self._config._julius_dict_htkconf_en])
        self._cmdline.extend(["-b", "1500", "-b2", "100",
                                    "-s", "500" ,"-m", "10000"])
        self._cmdline.extend(["-n", "30", "-output", "5", "-zmeanframe",
                                  "-rejectshort" ,"800", "-lmp", '10' ,'0',
                                  '-lmp2', '10', '0'])

      else:
        rospy.logerr("Invalid setrings")
        return False

    else:
      #
      #  Japanese
      if self._lang in ('ja', 'jp'):
        self._cmdline.extend(['-h',  self._config._julius_hmm_ja])
        self._cmdline.extend(['-hlist', self._config._julius_hlist_ja])
        self._cmdline.extend(["-dfa", os.path.join(self._config._basedir, "etc", "dummy.dfa")])
        self._cmdline.extend(["-v" , os.path.join(self._config._basedir, "etc", "dummy.dict")])
        self._cmdline.extend(["-sb", "80.0"])
      #
      #  Germany
      elif self._lang == 'de':
        self._cmdline.extend(['-h',  self._config._julius_hmm_de])
        self._cmdline.extend(['-hlist', self._config._julius_hlist_de])
        self._cmdline.extend(["-dfa", os.path.join(self._config._basedir, "etc", "dummy-en.dfa")])
        self._cmdline.extend(["-v", os.path.join(self._config._basedir, "etc", "dummy-en.dict")])
        self._cmdline.extend(["-sb", "160.0"])
      #
      #  English
      else:
        self._cmdline.extend(['-h',  self._config._julius_hmm_en])
        self._cmdline.extend(['-hlist', self._config._julius_hlist_en])
        self._cmdline.extend(["-dfa", os.path.join(self._config._basedir, "etc", "dummy-en.dfa")])
        self._cmdline.extend(["-v", os.path.join(self._config._basedir, "etc", "dummy-en.dict")])
        self._cmdline.extend(["-sb", "160.0"])

      #
      #  large model or small model
      #
      if self._memsize == "large":
        self._cmdline.extend(["-b", "-1", "-b2", "120", 
                                  "-s", "1000" ,"-m", "2000"])
      else:
        self._cmdline.extend(["-b", "-1", "-b2", "80",
                                 "-s", "500" ,"-m", "1000"])
    
      self._cmdline.extend(["-n", "5", "-output", "5"])
      self._cmdline.extend(["-rejectshort", "200"])
      # (文法使用時) 第1,2パス用の単語挿入ペナルティ
      self._cmdline.extend(["-penalty1", "5.0", "-penalty2", "20.0"])

    ######## 共通設定 ######################
    # レベル・零交差による音声区間検出の強制ON
    self._cmdline.extend(["-pausesegment"])
    # ゼロ続きの無効な入力部の除去をOFFにする
    self._cmdline.extend(["-nostrip"])
    # ショートポーズ音響モデルの名前
    self._cmdline.extend(["-spmodel", "sp"])
    # 第1パスの単語間トライフォン計算法を指定する．
    # (同じコンテキストのトライフォン集合の全尤度の最大値を近似尤度として用いる)
    self._cmdline.extend(["-iwcd1", "max"])
    # safe pruning 上位N個が確実に求まる．正確．
    self._cmdline.extend(["-gprune", "safe"])
    # エラー単語を無視して続行する
    self._cmdline.extend(["-forcedict"])
    # 認識した音声データを連続したファイルに自動保存
    self._cmdline.extend(["-record", self._logdir])
    # サンプリング周波数(Hz)
    self._cmdline.extend(["-smpFreq", "16000"])

    # 入力の設定（adinport使用)
    self._audioport = self.getunusedport()
    self._cmdline.extend(["-input", "adinnet",
            "-adport", str(self._audioport)])

    if self._jconf_file :
         # overwrite parameters by jconf file.
        self._cmdline.extend(["-C", self._jconf_file])

    self._moduleport = self.getunusedport()
    # module mode
    self._cmdline.extend(["-module", str(self._moduleport)])

    return True

  #
  #  Connect to Julius
  def connect_to_julius(self, host, port):
    if not self._modulesocket :
      self._modulesocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      self._modulesocket.connect((host, port))
      self._modulehost = host
      self._moduleport = port
      return Ture
    except socket.error:
      return False

  #
  #  close Julius
  def close_julius(self):
    if self._modulesocket :
      try:
        self._modulesocket.sendall("DIE\n".encode('utf-8'))
        time.sleep(1)
        self._modulesocket.shutdown(socket.RDWR)
        self._modulesocket.close()
      except:
        pass
      self._modulesocket = None
    return

  #
  #  Connect to Adinnet
  def connect_to_adinnet(self, host, port):
    if not self._audiosocket :
      self._audiosocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      self._audiosocket.connect((host, port))
      self._audiohost = host
      self._audioport = port
      return True
    except socket.error:
      return False

  #
  #  close Adinnet
  def close_adinnet(self):
    if self._audiosocket :
      try:
        self._audiosocket.shutdown(socket.RDWR)
        self._audiosocket.close()
      except:
        pass
      self._audiosocket = None
    return


  #
  #  get unused communication port
  #
  def getunusedport(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', 0))
    addr, port = s.getsockname()
    s.close()
    return port

  #
  #  Terminate (Call on Finished)
  #
  def terminate(self):
    rospy.loginfo ('JuliusWrap: terminate')
    self._running = False
    self.close_adinnet()
    self.close_julius()
    if self._p :
      self._p.terminate()
    return 0

  #
  #   Write to audio data
  #
  def write(self, data):
    try:
      if len(data) > 0:
        self._audiosocket.send(struct.pack("i", len(data)))
        self._audiosocket.sendall(data)
    except socket.error:
      try:
        self._audiosocket.connect((_audiohost, self._audioport))
      except:
        pass
    return 0

  #
  #  Run
  #
  def run(self):
    while self._running:
      for f in glob(os.path.join(self._logdir, "*.wav")):
        for c in self._callbacks:
          c(self.CB_LOGWAVE, f)

      try:
        self._modulesocket.settimeout(1)
        if sys.version_info.major == 2:
          data = self._prevdata + unicode(self._modulesocket.recv(1024*10),  self._jcode)
        else:
          data = self._prevdata + self._modulesocket.recv(1024*10).decode(self._jcode)
      except socket.timeout:
        continue
      except socket.error:
        rospy.logerr ('socket error')
        break

      #print("==>",data)
      self._gotinput = True
      ds = data.split(".\n")
      self._prevdata = ds[-1]
      ds = ds[0:-1]
      for d in ds:
        try:
          dx = BeautifulSoup(d, "lxml")
          for c in self._callbacks:
            c(self.CB_DOCUMENT, dx)
        except:
          traceback.print_exc()
          pass

    rospy.loginfo ('JuliusWrap: exit from event loop')
    return

  #
  #   Add grammer to Julius Server
  #
  def addgrammar(self, data, name):
    if self._firstgrammar == True:
      val="CHANGEGRAM %s\n" % (name,)
      self._modulesocket.sendall(val.encode('utf-8'))
      self._firstgrammar = False
    else:
      val="ADDGRAM %s\n" % (name,)
      self._modulesocket.sendall(val.encode('utf-8'))
    self._modulesocket.sendall(data.encode(self._jcode, 'backslashreplace'))
    self._grammars[name] = len(self._grammars)
    self._activegrammars[name] = True
    time.sleep(0.1)
    return

  #
  #  Activate current grammer
  #
  def activategrammar(self, name):
    try:
      gid = self._grammars[name]
    except KeyError:
      rospy.logerr ("[error] unknown grammar: %s" % (name,))
      return
    val="ACTIVATEGRAM %s\n" % (name,)
    self._modulesocket.sendall(val.encode('utf-8'))
    self._activegrammars[name] = True
    time.sleep(0.1)
    return

  #
  #  Deactivate current grammer
  #
  def deactivategrammar(self, name):
    try:
      gid = self._grammars[name]
    except KeyError:
      rospy.logerr ("[error] unknown grammar: %s" % (name,))
      return
    val="DEACTIVATEGRAM %s" % (name,)
    self._modulesocket.sendall(val.encode('utf-8'))
    del self._activegrammars[name]
    time.sleep(0.1)
    return

  #
  #  Synchronize grammer
  #
  def syncgrammar(self):
    self._modulesocket.sendall("SYNCGRAM\n".encode('utf-8'))
    return

  #
  #  Switch grammer
  #
  def switchgrammar(self, name):
    keys=list(self._activegrammars.keys())
    for g in keys:
      if g != name:
        self.deactivategrammar(g)
    self.activategrammar(name)
    return

  #
  #  Set callback function
  #
  def setcallback(self, func):
    self._callbacks.append(func)
    return

######################################################
#
#  JuliusRos
#
class JuliusRos(openhri.OpenHRI_Component):
  #
  #  Constructor
  #
  def __init__(self, manager, grammar=''):
    openhri.OpenHRI_Component.__init__(self, manager)
    self._lang = 'ja'
    self._srgs = None
    self._j = None
    self._mode = 'grammar'
    self.params = {}
    self._audio_topic = '/audio_capture/audio'
    self._result_topic = '/julius/result'
    self._result_raw_topic = '/julius/result_raw'
    self._result_threshold = 0.5
    self._jconf_file="main.jconf"

    self._properties = None
    self._grammar_name=grammar

    self._copyrights.append( read_file_contents(os.path.join( self._config._basedir, "doc", "julius_copyright.txt")))
    self._copyrights.append( read_file_contents(os.path.join( self._config._basedir, "doc", "voxforge_copyright.txt")))

  #
  #  OnInitialize
  #
  def onInitialize(self):
    rospy.loginfo("JuliusRos version " + __version__)
    rospy.loginfo("Copyright (C) 2010-2011 Yosuke Matsusaka, AIST")
    rospy.loginfo("Copyright (C) 2017-2019 Isao Hara, AIST")
    rospy.loginfo("Copyright (C) 2020 Isao Hara, RT-Coorp.")

    ###############
    #  setup ports
    self._audio_sub = rospy.Subscriber(self._audio_topic, AudioData, self.onData)
    self._julius_result = rospy.Publisher(self._result_topic, String, queue_size=10)
    self._julius_raw_result = rospy.Publisher(self._result_raw_topic, String, queue_size=10)


    #
    # Parameters
    self._jconf_file=self.bindParameter("jconf_file", None, "main.jconf")
    self._audio_topic=self.bindParameter("audio_topic", None, self._audio_topic)
    self._result_topic=self.bindParameter("result_topic", None, self._result_topic)
    self._result_raw_topic=self.bindParameter("result_raw_topic", None, self._result_raw_topic)

    self.show_copyrights()

    return True

  #
  # call by manager.shutdown
  def onShutdown(self):
    print("\n]]]] Shutdown Julius: %s" % self._grammar_name)
    self.kill_julius()
    return

  #
  #  OnActivate
  #
  def onActivated(self, ec_id=0):
    if self._mode == 'dictation' :
      pass
      #self._lang = 'ja'
    else:
      self._lang = self._srgs._lang

    self._j = JuliusWrap(self._lang, self)
    if not self._j._running :
      return False

    self._j.start()
    self._j.setcallback(self.onResult)

    while self._j._gotinput == False:
      time.sleep(0.1)

    if self._j._mode == 'dictation' :
      rospy.loginfo("run with dictation mode")

    else:
      for r in self._srgs._rules.keys():
        gram = self._srgs.toJulius(r)
        if gram == "":
          return False
        rospy.loginfo("register grammar: %s" % (r,))
        self._j.addgrammar(gram, r)
      self._j.switchgrammar(self._srgs._rootrule)

    return True

  #
  #  OnDeactivate
  #
  def onDeactivate(self, ec_id):
    self.kill_julius()
    return True

  ##############################################
  #  OnData (Callback for Subscriber)
  #
  def onData(self, msg):
    if self._j:
      if len(msg.data) > 0: self._j.write(msg.data)
    return

  #
  #
  def onCommand(self, msg):
    if self._j:
      self._j.switchgrammar(msg.data)
    return

  #
  #  OnResult (call by JuliusWrapper)
  #
  def onResult(self, type, data):
    if type == JuliusWrap.CB_DOCUMENT:
      if data.input:
        d=data.input
        rospy.loginfo(d['status'])
        #self._statusdata.data = str(d['status'])
        #self._statusport.write()

      elif data.rejected:
        d=data.rejected
        rospy.loginfo('rejected')
        #self._statusdata.data = 'rejected'
        #self._statusport.write()

      elif data.recogout:
        d = data.recogout
        doc = Document()
        listentext = doc.createElement("listenText")
        doc.appendChild(listentext)
        result_raw=None
        for s in d.findAll('shypo'):
          hypo = doc.createElement("data")
          score = 0
          count = 0
          text = []
          for w in s.findAll('whypo'):
            if not w['word'] or  w['word'][0] == '<':
              continue
            whypo = doc.createElement("word")
            whypo.setAttribute("text", w['word'])
            whypo.setAttribute("score", w['cm'])
            hypo.appendChild(whypo)
            text.append(w['word'])
            score += float(w['cm'])
            count += 1
          if count == 0:
            score = 0
          else:
            score = score / count

          hypo.setAttribute("rank", s['rank'])
          hypo.setAttribute("score", str(score))
          hypo.setAttribute("likelihood", s['score'])
          hypo.setAttribute("text", " ".join(text))

          if (result_raw is None) and score > self._result_threshold:
            result_raw = "".join(text).encode('utf-8')
            #print(result_raw)
            self._julius_raw_result.publish(result_raw)

          if PYTHON_MAJOR_VERSION == 2:
            rospy.loginfo("#%s: %s (%s)" % (s['rank'], " ".join(text).encode('utf-8'), str(score)))
          else:
           rospy.loginfo("#%s: %s (%s)" % (s['rank'], " ".join(text),str(score)))

          listentext.appendChild(hypo)


        data = doc.toxml(encoding="utf-8")
        self._julius_result.publish(data.decode('utf-8'))

        #rospy.loginfo(data.decode('utf-8', 'backslashreplace'))
        #self._outdata.data = data.decode('unicode_escape')
        #self._outport.write()

    elif type == JuliusWrap.CB_LOGWAVE:
      t = os.stat(data).st_ctime
      tf = t - int(t)
      #self._logdata.tm = RTC.Time(int(t - tf), int(tf * 1000000000))
      try:
        wf = wave.open(data, 'rb')
        self._logdata.data = wf.readframes(wf.getnframes())
        wf.close()
        os.remove(data)
        self._logport.write()
      except:
        pass

    return

  #
  #  Set Grammer
  #
  def setgrammar(self, srgs):
    self._srgs = srgs
    return

  #
  #  Set Grammer
  #
  def setgrammarfile(self, gram, rebuid=False):
    if not os.path.exists(gram) :
       gram=os.path.join(self._config._basedir, gram)
    self._grammer = gram
    rospy.loginfo("compiling grammar: %s" % (gram,))
    self._srgs = SRGS(gram, self._properties, rebuid)
    rospy.loginfo ("done")
    return

  #
  # Kill subprocess
  def kill_julius(self):
    if self._j:
      self._j.terminate()
      self._j.join()
      self._j = None
    return

#
#  JuliusRTCManager Class
#
class JuliusRosManager(openhri.OpenHRI_Manager):
  #
  #  Constructor
  #
  def __init__(self):
    openhri.set_manager_info(__version__, "%prog [srgfile]", __doc__)

    openhri.OpenHRI_Manager.__init__(self, name='JuliusRos',
                                     conf_name='julius.conf')

    #
    #
    if self._opts.guimode == True:
      sel = askopenfilenames(title="select W3C-SRGS grammar files")
      if sel is not None:
        self._args.extend(sel)
    
    if self._opts.dictation_mode == False and len(self._args) == 0:
      self._parser.error("wrong number of arguments")
      sys.exit(1)
           
    if self._opts.dictation_mode == True:
      self._args.extend(['dictation'])

    #
    #
    self._rebuid_lexicon= self._opts.rebuild_lexicon
    self._grammars = self._args
    self._config = config(config_file=self._opts.configfile)

  #
  #
  def add_options(self, parser):
    parser.add_option('-g', '--gui', dest='guimode', action="store_true",
                      default=False,
                      help='show file open dialog in GUI')

    parser.add_option('-D', '--dictation', dest='dictation_mode', 
                      action="store_true",
                      default=False,
                      help='run with dictation mode')

    parser.add_option('-r', '--rebuild-lexicon', dest='rebuild_lexicon',
                      action="store_true",
                      default=False,
                      help='rebuild lexicon')
    return



  #
  #  Initialize node
  #
  def moduleInit(self):
    for a in self._grammars:
      rospy.loginfo("Create JuliusRos: %s", a)
      self._comp[a]=self.create_component(JuliusRos, a)
      if a is None:
        return
      elif a == 'dictation':
        self._comp[a]._mode='dictation'
        self._comp[a]._lang = 'ja'
      elif a == 'dictation_en':
        self._comp[a]._mode='dictation'
        self._comp[a]._lang='en'
      else:
        self._comp[a].setgrammarfile(a, self._rebuid_lexicon)
    return

#
#
g_manager = None

def main():
  g_manager = JuliusRosManager()
  g_manager.init_node()
  g_manager.start()

  g_manager.shutdown()
  return


#
#  Main
#
if __name__=='__main__':
    main()

