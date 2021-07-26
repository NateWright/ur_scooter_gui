#!/usr/bin/env/ python3

# Use this one for python2
# #!/usr/bin/env/ python2

import rospy
from simple_ui import Scooter
from arm_laser import ArmLaser
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray

class GuiController():
    """
    Controls state of system. Takes a state message from /gui_state in the form of a string. Blocks state form updating
    while the current state is still executing
    """
    def __init__(self):
        self.state = "drive"
        self.prev_state = None
        self.desired_state = None

        self.scooter = Scooter()
        self.state_sub = rospy.Subscriber("desired_state", String, self.state_sub_cb)
        self.state_sub = rospy.Subscriber("point", UInt32MultiArray, self.point_sub_cb)
        self.rate = rospy.Rate(30)

        self.sample_points = None
        self.center = None

    def state_sub_cb(self, data):
        self.desired_state = data
        print("Need to implement state_sub_cb")

    def point_sub_cb(self, data):
        self.desired_state = data.data
        print("Image Pixel Selected: X={} Y={}".format(data.data[0], data.data[1]))

    def update_state(self):
        # TODO Implement state machine calls
        print("Implement update state")

    def drive(self):
        self.scooter.go_to_travel_config()
        self.scooter.set_gripper(False)
        self.prev_state = self.state
        if self.desired_state == "gather_pick_cloud":
            self.state = "gather_pick_cloud"

    def gather_pick_cloud(self):
        self.scooter.go_to_fold_config()

        if self.scooter.skip_grasp and self.scooter.has_saved_pointcloud():
            self.scooter.publish_saved_pointcloud()
        else:
            self.scooter.update_pointcloud()
            self.scooter.save_pointcloud()

        self.prev_state = self.state
        self.state = "object_selection"

    def object_select(self):
        if self.point is not None:
            # TODO Implement the segmentation code based on the point
            self.prev_state = self.state
            self.state = "segment_object"
            #TODO Make sure we can publish self.center
            self.center = None

    def segment_object(self):
        print("Selection made at:{}".format(self.center))
        self.sample_points = self.scooter.get_sample_points(self.center)

        if len(self.sample_points) == 0:
            self.sample_points = self.scooter.get_sample_points(self.center)

        self.prev_state = self.state
        self.state = "object_confirmation"

    def object_confirmation(self):
        if self.desired_state == "pick_object":
            self.prev_state = self.state
            if self.scooter.skip_grasp:
                self.scooter.clear_sample_points()
                self.state = "drive"
            else:
                self.state = "pick_object"

    def pick_object(self):
        grasps = self.scooter.get_reachable_grasps(self.sample_points)
        grasp = self.scooter.automatic_grasp_selection(grasps, self.sample_points, self.center)

        if grasp is not None:
            self.scooter.pick_object(grasp)
            self.prev_state = self.state
            if self.scooter.object_detected():
                self.state = "basket"
            else:
                self.state = "drive"

    def basket(self):
        self.scooter.place_object_to_basket()
        self.prev_state = self.state
        self.state = "drive"

    def run(self):
        while not rospy.is_shutdown():
            self.update_state()
            self.rate.sleep()


def main():
    gui = GuiController
    gui.run()

if __name__ == "__main__":
    main()