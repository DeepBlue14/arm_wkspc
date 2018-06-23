# [db] 2012.04.13
# dan@cs.uml.edu
import threading
import copy
class SharedBufferValue(object):
    """ Safe single value buffering class"""
    def __init__(self):
        self.__value = None
        self.__lock = threading.Lock()
        self.__newflag = False
    def has_new_value(self):
        """ Returns True if buffer holds a value """
        return self.__newflag
    def __set_value(self,value): 
        with self.__lock:
            self.__value = value
            self.__newflag = True
    def __get_value(self):
        with self.__lock:
            tmp = copy.deepcopy(self.__value) 
            self.__newflag = False
        return tmp
    value = property(__get_value,__set_value)
        