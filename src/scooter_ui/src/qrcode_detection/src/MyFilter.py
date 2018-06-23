#!/usr/bin/python 
# [db] 11/28/1012

# ThrottledSubscriber code by Dan Brooks
# 
# message_filters/__init__.py
# Original Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#import roslib
#roslib.load_manifest("qrcode_detection")
import rospy
#from message_filters import *
import threading
from MyRate import *
from SharedBufferValue import *

DEBUGGING = False
class SimpleFilter:
    def __init__(self):
        self.callbacks = {}
    def registerCallback(self, cb, *args):
        """
        Register a callback function `cb` to be called when this filter has output.
        The filter calls the function ``cb`` with a filter-dependent list of arguments,
        followed by the call-supplied arguments ``args``.
        """
        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))




class ThrottledSubscriber(SimpleFilter):
    def __init__(self, *args):
        SimpleFilter.__init__(self)
        self.topic = args[0]
        self.msg_type = args[1]
        self.timer = MyRate(args[2])
        self.data = SharedBufferValue()
        self.lock = threading.Lock()
        self.reader = rospy.Subscriber(self.topic,self.msg_type,self.receivedata)
        self.running = False
        self.thread = threading.Thread(target=self.sample_thread)
        self.thread.daemon = True
        self.thread.start()

    def __del__(self):
        self.running = False
        self.thread.join()

    def receivedata(self,data):
        with self.lock:
            self.data.value = data

    def sample_thread(self):
        self.running = True
        while not rospy.is_shutdown() and not self.data.has_new_value():
            self.timer.sleep()
        while not rospy.is_shutdown() and self.running:
            with self.lock:
                if self.data.has_new_value():
                    self.signalMessage(self.data.value)
            self.timer.sleep()

class PolledSubscriber(SimpleFilter):
    """ Non-blocking polled subscriber """
    def __init__(self, *args):
        SimpleFilter.__init__(self)
        self.topic = args[0]
        self.msg_type = args[1]
        self.data = SharedBufferValue()
        self.lock = threading.Lock()
        self.reader = rospy.Subscriber(self.topic,self.msg_type,self.receivedata)
        self.running = False

    def __del__(self):
        self.running = False
        self.thread.join()

    def receivedata(self,data):
        with self.lock:
            self.data.value = data

    def read(self):
        with self.lock:
            if not self.data.has_new_value():
                return None
            else:
                return self.data.value
                
    
class Subscriber(SimpleFilter):
    """
    ROS subscription filter.  Identical arguments as :class:`rospy.Subscriber`.
    This class acts as a highest-level filter, simply passing messages
    from a ROS subscription through to the filters which have connected
    to it.
    """
    def __init__(self, *args):
        SimpleFilter.__init__(self)
        self.topic = args[0]
        self.sub = rospy.Subscriber(*args, **{"callback" : self.callback})
    
    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic



class MySynchronizer(SimpleFilter):
    def __init__(self,fs,queue_size,accuracy):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.accuracy = accuracy
        self.lock = threading.Lock()
        self._reducesize = False;
        
    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [f.registerCallback(self.add, q) for (f,q) in zip(fs, self.queues)]

    def add(self,msg,my_queue):
        def decimalize(x):
            return float(x)*(pow(10,-len(str(x))))
        with self.lock:
            secs = msg.header.stamp.secs
            nsecs = msg.header.stamp.nsecs
            # To truncate precision, we just clip nsecs
            nsecs = int(str(nsecs)[:self.accuracy])
            t = secs+decimalize(nsecs)
            my_queue[t] = msg
            while len(my_queue) > self.queue_size:
                del my_queue[min(my_queue)]
            common = reduce(set.intersection, [set(q) for q in self.queues])
            for t in sorted(common):
                msgs = [q[t] for q in self.queues]
                self.signalMessage(*msgs)
                for q in self.queues:
                    del q[t]
            if self._reducesize:
                if DEBUGGING:
                    print " -- reducing size of queues -- "
                for q in self.queues:
                    target_size = len(q)/2
                    if DEBUGGING:
                        print "before: %d after %d"%(len(q),target_size)
                    while len(q) > target_size:
                        del q[min(q)]
                self._reducesize = False;
                if DEBUGGING:
                    print " -- done -- "

    def reducesize(self):
        self._reducesize = True
           
