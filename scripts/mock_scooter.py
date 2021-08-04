#!/usr/bin/env/ python3

# Use this one for python2
# #!/usr/bin/env/ python2

class Scooter:
    """
    Dummy Instance of the scooter used to test the GUI. Set the flags in the init to control the different scenarios
    """
    def __init__(self):
        print("Using Scooter Test Object")
        self.skip_grasp = False
        # self.skip_grasp = True

        # Used to tell if there is a saved pointcloud or not
        self.pointcloud = False
        # self.pointcloud = True

        # Dummy Points returned. Return length zero to do grasp by radius, and length one to do grasp by point
        # self.points = []
        self.points = [True]

        # Dummy variable containing a fake grasp
        # self.grasp = None   # If you want to force it to fail
        self.grasp = True   # If you want grasp to succeed

        # Dummy flag to tell if object was grasped successfully or not
        # self.object_grasped = False
        self.object_grasped = True

        self.printed_before = False

    def go_to_travel_config(self):
        if not self.printed_before:
            print("Moving to arm travel config")

    def set_gripper(self, flag):
        if not self.printed_before:
            print("Gripper set to {}".format(flag))
            self.printed_before = True

    @staticmethod
    def go_to_fold_config():
        print("Moving to fold config")

    def has_saved_pointcloud(self):
        return self.pointcloud

    @staticmethod
    def publish_saved_pointcloud():
        print("Publishing Saved Pointcloud")

    @staticmethod
    def update_pointcloud():
        print("Updating Pointcloud, Using Sensors and Stuff and Thangs")

    @staticmethod
    def update_pointcloud_2d_selection():
        print("Updating 2D Selection Pointcloud, Using Sensors and Stuff and Thangs")

    def save_pointcloud(self):
        print("Saving Pointcloud")
        self.pointcloud = True

    @staticmethod
    def center_from_2d_selection(point):
        print("Center at:{}".format(point))
        return point

    def get_sample_points(self, center):
        print("Getting sample points")
        return self.points

    @staticmethod
    def get_sample_points_by_radius(center):
        print("Getting sample points by radius")
        return [True]

    @staticmethod
    def clear_sample_points():
        print("Clearing sample points")
        # TODO Determine if this actually is meant to set the scooters instance of sample points to None

    @staticmethod
    def get_reachable_grasps(sample_points):
        print("Calculating reachable grasps")

    def automatic_grasp_selection(self, grasps, sample_points, center):
        print("Choosing grasp automatically")
        return self.grasp

    @staticmethod
    def pick_object(grasp):
        print("Picking object")

    def object_detected(self):
        print("Object detected")
        return self.object_grasped

    @staticmethod
    def place_object_to_basket():
        print("Placing in basket")
