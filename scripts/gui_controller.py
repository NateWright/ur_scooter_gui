#!/usr/bin/env python3

# Use this one for python2
###asdasd!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray

try:
    from simple_ui import Scooter
except ImportError:
    from mock_scooter import Scooter


class GuiController:
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

        # TODO Delete this because this relies on Sasha's code
        # This is the point from the gui image
        self.point = None   # Causes infinite loop in object_select
        # self.point = True   # Allows you to leave object_select

    def state_sub_cb(self, data):
        """
        Desired state published via the GUI Interface itself

        :param data: std_msgs.msg String
        """
        self.desired_state = data.data

    def point_sub_cb(self, data):
        """
        X,Y point published via the GUI Interface itself.

        :param data: std_msgs.msg UInt32MultiArray
        """
        self.point = data.data
        print("Image Pixel Selected: X={} Y={}".format(data.data[0], data.data[1]))

    def run_current_state(self):
        """
        Non-blocking function to activate the function that pertains to the current state. Each state will only execute
        a single time
        """
        if self.state != self.prev_state:
            if self.state == "drive":
                self.drive()
            elif self.state == "gather_pick_cloud":
                self.gather_pick_cloud()
            elif self.state == "object_select":
                self.object_select()
            elif self.state == "segment_object":
                self.segment_object()
            elif self.state == "object_confirmation":
                self.object_confirmation()
            elif self.state == "pick_object":
                self.pick_object()
            elif self.state == "basket":
                self.basket()

    def drive(self):
        """
        Stow the arm in front of the scooter. Wait until begin button is pressed in GUI for change
        """
        self.scooter.go_to_travel_config()
        self.scooter.set_gripper(False)
        if self.desired_state == "gather_pick_cloud":
            self.prev_state = self.state
            self.state = "gather_pick_cloud"

    def gather_pick_cloud(self):
        """
        Move the arm to the right side of the robot in stow position and proceed to alternate between the depth sensors
        to build a full image of the environment
        """
        self.scooter.go_to_fold_config()

        if self.scooter.skip_grasp and self.scooter.has_saved_pointcloud():
            self.scooter.publish_saved_pointcloud()
        else:
            # TODO Modified update_point_cloud to handle Sasha code
            # self.scooter.update_pointcloud()  # Removed for the 2D point version below
            self.scooter.update_pointcloud_2d_selection()
            self.scooter.save_pointcloud()

        self.prev_state = self.state
        self.state = "object_select"

    # TODO This is the function that will need to integrate with Sasha's stuff
    def object_select(self):
        if self.point is not None:
            print("This is where you should run your code to segment the object and set self.center")
            self.prev_state = self.state
            self.state = "segment_object"
            # TODO Determine if this is meant to go up to 640 or 639
            self.center = self.scooter.center_from_2d_selection(self.point)
            # TODO Implement Image back propagation from point cloud so we can ask user if this is the correct object

    def segment_object(self):
        """
        System segments the object found at the center point, and segments the object at the center. That depth points
        for that point are sent on to the grasping algorithm. If unable to find an object at that point it will draw a
        sphere in euclidean space and send all depth points within that sphere onto the grasp selection algorithm.
        """
        print("Selection made at:{}".format(self.center))
        self.sample_points = self.scooter.get_sample_points(self.center)

        if len(self.sample_points) == 0:
            self.sample_points = self.scooter.get_sample_points_by_radius(self.center)

        self.prev_state = self.state
        self.state = "object_confirmation"

    def object_confirmation(self):
        """
        Wait for the GUI to prompt for a full pick. If it's a full trial it will grasp the object. If not then we clear
        the previous globals and return to drive
        """
        if self.desired_state == "pick_object":
            self.prev_state = self.state
            if self.scooter.skip_grasp:
                # Clear the no longer needed globals for partial trials
                self.scooter.clear_sample_points()
                self.state = "drive"
            else:
                self.state = "pick_object"
        # TODO Implement denial logic on the gui side if we want to do that

    def pick_object(self):
        """
        System segments all reachable grasps within the points given to it. Automatically determines what the grasp that
        will be utilized and performs it. If grasp succeeds proceeds to basket, else if grasp fails it will return to
        drive. Grasp success is determined based on the gripper feedback
        """
        grasps = self.scooter.get_reachable_grasps(self.sample_points)
        grasp = self.scooter.automatic_grasp_selection(grasps, self.sample_points, self.center)

        if grasp is not None:
            self.scooter.pick_object(grasp)
            self.prev_state = self.state
            if self.scooter.object_detected():
                self.state = "basket"
            else:
                self.state = "drive"
        else:
            self.state = "drive"

    def basket(self):
        """
        Move the arm to hover over the basket and proceed to drop the object. The system then proceeds to its initial
        drive config
        """
        self.scooter.place_object_to_basket()
        self.prev_state = self.state
        self.state = "drive"

    def run(self):
        """
        Runs the state machine and rests between iterations at the frequency determined by rate
        """
        while not rospy.is_shutdown():
            self.run_current_state()
            self.rate.sleep()


def main():
    print("GUI Controller Spinning Up")
    rospy.init_node("Gui_controller", anonymous=True)
    gui = GuiController()
    gui.run()


if __name__ == "__main__":
    main()
