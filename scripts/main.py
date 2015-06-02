#!/usr/bin/python
# example1.py
# simple point tracking program

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from OWL import *

# Phasespace configuration
MARKER_COUNT = 4
SERVER_NAME = "192.168.1.138"
INIT_FLAGS = 0

# ROS Globals
group = None
scene = None
PLANNING_FRAME = "/base"
collision_publisher = None
obj_dict = {
             "default": {"size": [0.4, 0.4, 0.4]},
             "workbench_table": {"size": [1.8288, 0.772, 0.075], "position": [0, -0.361, 0.]}
           }

def main():
  global collision_publisher, group, scene, PLANNING_FRAME
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('phasespace_tracker')
  collision_publisher = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)
  rospy.sleep(2)

  group = moveit_commander.MoveGroupCommander("both_arms")
  scene = moveit_commander.PlanningSceneInterface()
  PLANNING_FRAME = group.get_planning_frame()
  print "Planning frame set to ", PLANNING_FRAME

  init_phasespace()
  main_loop()

def init_phasespace():
  global SERVER_NAME, MARKER_COUNT, INIT_FLAGS
  if(owlInit(SERVER_NAME, INIT_FLAGS) < 0):
    print "init error: ", owlGetError()
    sys.exit(0)

  # create tracker 0
  tracker = 0
  owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER)

  # set markers
  for i in range(MARKER_COUNT):
      owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i)

  # activate tracker
  owlTracker(tracker, OWL_ENABLE)

  # flush requests and check for errors
  if(owlGetStatus() == 0):
      owl_print_error("error in point tracker setup", owlGetError())
      sys.exit(0)

  # set define frequency
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY)

  # start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE)


def load_id_config_file(self):
  '''
    TODO: 
      Load a config file associating marker IDs with object types and sizes
  '''
  pass

def setup_scene():
  print "Setting up scene..."
  add_table() # Workbench is static, should always be included in the scene

def main_loop():
  setup_scene() # Attach any known baseline objects to the scene
  print "Scene set."

  while not rospy.is_shutdown():
    markers = []

    # get some markers
    markers = owlGetMarkers()
    n = markers.size()
    # check for error
    err = owlGetError()
    if(err != OWL_NO_ERROR):
        owl_print_error("error", err)
        break

    # no data yet
    if(n == 0): continue

    if(n > 0):
      print n, "markers:"
      for i in range(n):
        if(markers[i].cond > 0):
          print "%d) %.2f %.2f %.2f" % (i, markers[i].x, markers[i].y, markers[i].z)
          add_object (i, markers[i].x, markers[i].y, markers[i].z)
      print ""

  # 
  owlDone();


def add_table():
  '''
  Add the workbench table to the planning scene
  '''
  add_object("workbench_table", 0, 0, 0)

def add_object(id,x,y,z):
  global obj_dict, collision_publisher, PLANNING_FRAME
  x /= 1000
  y /= 1000
  z /= 1000

  #phasespace Y and Z are swapped:
  ytemp = y
  y = z
  z = ytemp
  
  collision_object = moveit_msgs.msg.CollisionObject()
  collision_object.id = "obj"+str(id)
  collision_object.header.frame_id = PLANNING_FRAME
  sp = shape_msgs.msg.SolidPrimitive()
  sp.type = shape_msgs.msg.SolidPrimitive.BOX

  pose = geometry_msgs.msg.Pose()
  pose.orientation.w = 1.
  pose.orientation.x = 0.
  pose.orientation.y = 0.
  pose.orientation.z = 0.

  if not id in obj_dict:
    sp.dimensions = obj_dict["default"]["size"]
    pose.position.x, pose.position.y, pose.position.z = x,y,z    
  else:
    sp.dimensions = obj_dict[id]["size"]
    if not "position" in obj_dict[id]:
      pose.position.x, pose.position.y, pose.position.z = x,y,z
    else:
     pose.position.x, pose.position.y, pose.position.z = obj_dict[id]["position"]

  collision_object.primitives.append(sp)
  collision_object.primitive_poses.append(pose)
  collision_object.operation = collision_object.ADD
  
  print collision_object
  print collision_publisher
  collision_publisher.publish(collision_object)


def owl_print_error(s, n):
  """Print OWL error."""
  if(n < 0): print "%s: %d" % (s, n)
  elif(n == OWL_NO_ERROR): print "%s: No Error" % s
  elif(n == OWL_INVALID_VALUE): print "%s: Invalid Value" % s
  elif(n == OWL_INVALID_ENUM): print "%s: Invalid Enum" % s
  elif(n == OWL_INVALID_OPERATION): print "%s: Invalid Operation" % s
  else: print "%s: 0x%x" % (s, n)


if __name__ == "__main__":
  main()