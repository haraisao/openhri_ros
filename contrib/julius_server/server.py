# -*- coding: utf-8 -*-
#
#  PyWebScok Library
#  Communication Adaptor for WebSocket
#
#   Copyright(C) 2015, Isao Hara, AIST, All Right Reserved
#   Licensed under the MIT License.
#   http://www.opensource.org/licenses/MIT
#

################################
from __future__ import print_function
import sys
import os
import signal
import comm
import julius_wrap
from daemon import DaemonContext

global __srv

def signalHandler(sig, handler):
   global __srv
   print( "Call sig")

   if __srv :
     __srv.terminate()
   else:
     print( "No service found" )
   sys.exit()
######################################

def daemonize():
  def fork():
    pid = os.fork()
    if pid > 0:
      f = open('/var/run/julius_server.pid','w')
      f.write(str(pid)+"\n")
      f.close()
      sys.exit()

  def throw_away_io():
    stdin = open(os.devnull, 'rb')
    stdout = open(os.devnull, 'ab+')
    stderr = open(os.devnull, 'ab+', 0)

    for (null_io, std_io) in zip((stdin, stdout, stderr),
                                 (sys.stdin, sys.stdout, sys.stderr)):
      os.dup2(null_io.fileno(), std_io.fileno())

  fork()
  os.setsid()
  fork()
  throw_away_io() 

###########################################################
# Julius Server
#
def main(num=10000, top="html", host="", ssl=False, make_thread=True):
  global __srv
  if type(num) == str: num = int(num)
  reader = comm.HttpReader(None, top)
  reader.asr = julius_wrap.JuliusWrap()
  reader.asr.startJulius()
  reader.asr.start()

  __srv=comm.SocketServer(reader, "JuliusServer", host, num, ssl)
  signal.signal(signal.SIGINT,  signalHandler)
  signal.signal(signal.SIGTERM,  signalHandler)
  if make_thread :
    __srv.start()
  else:
    __srv.main_loop() 

  return __srv

def main2(num=10000):
  main(num=num, make_thread=False)

#
#
if __name__ == '__main__' :
  if '-d' in sys.argv:
     daemonize()
  main2()

