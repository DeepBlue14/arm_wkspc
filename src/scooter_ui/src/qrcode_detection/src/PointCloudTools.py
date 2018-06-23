# [db] 2012.04.30
# dan@cs.uml.edu

import roslib
#roslib.load_manifest('python_msg_conversions')
import rospy
import numpy as np
import threading
import ctypes,math,struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

#Doesn't exist, was only in ROS Fuerte
#from python_msg_conversions import pointclouds
#I downloaded a local copy
import pointclouds

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def create_cloud(header, fields, points):
    
    cloud_struct = struct.Struct(_get_struct_fmt(False,fields))
    buff = ctypes.create_string_buffer(cloud_struct.size*len(points))
    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff,offset,*p)
        offset += point_step
    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size*len(points),
                       data=buff.raw)

def create_cloud_xyz(header,points):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4,PointField.FLOAT32, 1),
              PointField('z', 8,PointField.FLOAT32, 1)]
    return create_cloud(header,fields,points)

def read_cloud_xyz(cloud):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4,PointField.FLOAT32, 1),
              PointField('z', 8,PointField.FLOAT32, 1)]
    assert(cloud)
    skip_nans = False
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields)
    width, height, point_step, row_step, data, unpack_from, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, struct.unpack_from, math.isnan
    if skip_nans:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                p = unpack_from(fmt, data, offset)
                has_nan = False
                for v in p:
                    if isnan(v):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
                offset += point_step
    else:
        for v in xrange(height):
            offset = row_step*v
            for u in xrange(width):
                yield unpack_from(fmt, data, offset)
                offset += point_step

# reads index, and 1 pixel in all 4 directions of index
def read_offset_and_alternatives(cloud_arr, x, y, w, h):
    for df in range(0,20):
        ps=[(x,y+df),(x,y-df),(x+df,y),(x-df,y)]
        i=0
        for pss in ps:
            if pss[0] >= 0 and pss[0] < cloud_arr.shape[0] and pss[1]>=0 and pss[1]<cloud_arr.shape[1]: 
#                print pss
                P=cloud_arr[pss[1],pss[0]]
#                print P
                for p in (P['x'], P['y'], P['z']):
                    if math.isnan(p):
                        i=-1
                        break
                if i != -1:
                    return P
            if df==0:
                break
    return None

def read_offset(cloud, index):
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, None)
    offset = index*cloud.point_step
    return struct.unpack_from(fmt, cloud.data, offset)


def read_points(cloud, field_names=None, skip_nans=False):
    assert(cloud)
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, unpack_from, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, struct.unpack_from, math.isnan
    if skip_nans:
        for v in xrange(height):
            offset = row_step * v
            for u in xrange(width):
                p = unpack_from(fmt, data, offset)
                has_nan = False
                for v in p:
                    if isnan(v):
                        has_nan = True
                        print "nan"
                        break
                if not has_nan:
                    yield p
                offset += point_step
    else:
        for v in xrange(height):
            offset = row_step*v
            for u in xrange(width):
                yield unpack_from(fmt, data, offset)
                offset += point_step



def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length
    return fmt
