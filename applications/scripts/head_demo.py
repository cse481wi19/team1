#! /usr/bin/env python

import rospy
import robot_api
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

def print_usage():
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    print 'Usage:'
    print '    rosrun applications head_demo.py look_at FRAME_ID X Y Z'
    print '    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG'
    print '    rosrun applications head_demo.py eyes ANG'
    print '    rosrun applications head_demo.py [nod|shake]_head'
    print '    rosrun applications head_demo.py be_[happy|sad|neutral]'
    print 'Examples:'
    print '    rosrun applications head_demo.py look_at base_link 1 0 0.3'
    print '    rosrun applications head_demo.py pan_tilt 0 0.707'
    print '    rosrun applications head_demo.py eyes .50'
    print '    rosrun applications head_demo.py nod_head'
    print '    rosrun applications head_demo.py be_happy'


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('head_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    head = robot_api.Head()
    expression = robot_api.Expressions()

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id, x, y, z = argv[2], float(argv[3]), float(argv[4]), float(
            argv[5])
        
        # Setup Header
        h = Header()
        h.frame_id = frame_id
        h.stamp = rospy.Time().now()

        # Setup Point
        p = Point()
        p.x = x
        p.y = y
        p.z = z

        # Point Stamped
        ps = PointStamped()
        ps.header = h
        ps.point = p

        result = head.look_at(ps, True)

        if result: print("Success")
        else: print("Point out of range")

    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan, tilt = float(argv[2]), float(argv[3])
        head.pan_and_tilt(pan, tilt)
    elif command == 'eyes':
        if len(argv) < 3:
            print_usage()
            return
        angle = float(argv[2])
        head.eyes_to(angle)
    elif command == 'nod_head':
        expression.nod_head()
    elif command == 'shake_head':
        expression.shake_head()
    elif command == 'be_happy':
        expression.be_happy()
    elif command == 'be_sad':
        expression.be_sad()
    elif command == 'be_neutral':
        expression.be_neutral()
    else:
        print_usage()


if __name__ == '__main__':
    main()
