import subprocess

import rospy

if __name__ == '__main__':
    rospy.init_node('Camera', anonymous=True)

    id_param = rospy.search_param('id')
    if not id_param or not uri_param: 
        print("No ID or URI Specified! Abort.")
        sys.exit(0)

    ID = int(rospy.get_param(id_param, '0'))

    py2_cmd = "python2 py2_cam_node.py " + str(ID)

    p = subprocess.Popen(py2_cmd.split())
