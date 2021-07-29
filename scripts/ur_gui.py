#!/usr/bin/env python

# This next one for python2
# #!/usr/bin/env python2

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import UInt32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os

success = None
image_required = False
update_rate = 50  # In milliseconds
zoom_coordinate = None
circle_coordinate = None
image = None
scale_factor = 1.5  # Scale factor for all assets relative to the native 640x480 image
directory = os.path.dirname(__file__)
large_font = ("Helvetica", 60, "bold")
small_font = ("Helvetica", 36)


# TODO Font Adjusting

class SampleApp(tk.Tk):
    """
    Main TK Instance - Controls the page frame of the system
    """

    def __init__(self):
        tk.Tk.__init__(self)
        self._frame = None
        self.switch_frame(StartPage)

    def switch_frame(self, frame_class):
        """Destroys current frame and replaces it with a new one."""
        new_frame = frame_class(self)
        if self._frame is not None:
            self._frame.destroy()
        self._frame = new_frame
        self._frame.pack()


class StartPage(tk.Frame):
    """
    Home Page - Only hosts the begin button. Begin button resets the global variables
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.button = tk.Button(self, text="Begin", width=75, height=30, font=large_font,
                                activebackground="lime", bg="lime", command=lambda: master.switch_frame(PictureInitial))
        self.button.bind("<Button-1>", self.setup_globals)
        self.button.bind("<Button-1>", self.begin_button_pub, add="+")
        self.button.pack(fill="both", expand=True, padx=100, pady=100)

        master.after(update_rate, self.update())

    def update(self):
        """No necessary updates in this frame"""
        self.after(update_rate, self.update)

    @staticmethod
    def setup_globals(event):
        """
        Read the name

        :param event: Start Page Button Push
        :return: None
        """
        global success
        success = None
        global image_required
        image_required = True

    @staticmethod
    def begin_button_pub(event):
        log_pub.publish("begin, image_unzoomed, begin_button")
        desired_state_pub.publish("gather_pick_cloud")



class PictureInitial(tk.Frame):
    """
    This frame loads the initial unzoomed image taken from the camera, receives a user click, then zooms in on it.
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)

        self.frame = 0

        self.label = tk.Label(self, text="Please click the object on the touchscreen",
                              font=small_font).pack(side="top", fill="x",
                                                    pady=10)
        wheel_filename = os.path.join(directory, "../assets/loading_wheel.gif")
        self.picture = tk.PhotoImage(file=wheel_filename, format="gif -index {}".format(self.frame))
        self.image_label = tk.Label(self, image=self.picture)
        self.image_label.pack()
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        self.image_button = tk.Button(self, image=self.picture, command=lambda: master.switch_frame(PictureZoomed))

        master.after(update_rate, self.update())

    def update(self):
        """
        Deletes the loading wheel once system is ready, updates image and loads it. Then  it binds image to callback.
        Just spins wheel if system is waiting

        :return: None
        """

        if not image_required:
            self.image_label.destroy()
            image_unzoomed = os.path.join(directory, "../assets/unzoomed.png")
            self.picture.configure(file=image_unzoomed, format="png")
            self.image_button.pack(side="left")
            self.image_button.bind("<Button-1>", self.click)
            self.image_button.bind("<Button-1>", self.select_unzoomed_button_pub, add="+")
        else:
            self.wait_spin()
        self.after(update_rate, self.update)

    def wait_spin(self):
        """
        Spins the gif animation by altering the image frame

        :return: None
        """
        if self.frame < 16:
            self.frame = self.frame + 1
            wheel_filename = os.path.join(directory, "../assets/loading_wheel.gif")
            self.picture.configure(file=wheel_filename, format="gif -index {}".format(self.frame))
        else:
            self.frame = 0

    @staticmethod
    def click(event):
        """
        Currently only prints the mouseclick

        :param event: Mouseclick on Image
        :return: None
        """
        global zoom_coordinate
        zoom_coordinate = (event.x, event.y)

    @staticmethod
    def select_unzoomed_button_pub(event):
        log_pub.publish("image_unzoomed, image_zoomed, image_button")


class PictureZoomed(tk.Frame):
    """
    This frame loads the zoomed image via opencv, It Publishes the XY location on the image wherein the circle should be
    drawn by the PictureSelected Frame. The XY location is the pixel relative to the original unzoomed image
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)

        self.label = tk.Label(self, text="Please click the object on the zoomed in image", font=small_font).pack(
            side="top", fill="x",
            pady=10)

        self.zoom_point = zoom_coordinate

        unzoomed_filename = os.path.join(directory, "../assets/unzoomed.png")
        self.cv_image = cv2.imread(unzoomed_filename)
        self.x_dim = self.cv_image.shape[1]
        self.y_dim = self.cv_image.shape[0]

        self.x_limits = int(round(self.x_dim / 4))
        self.y_limits = int(round(self.y_dim / 4))

        self.x_region = [zoom_coordinate[0] - int(round((self.x_limits / 2 - 1))),
                         zoom_coordinate[0] + int(round((self.x_limits / 2)))]
        self.y_region = [zoom_coordinate[1] - int(round((self.y_limits / 2 - 1))),
                         zoom_coordinate[1] + int(round((self.y_limits / 2)))]

        if self.x_region[0] < 0:
            self.x_region = [0, self.x_limits - 1]
        elif self.x_region[1] > self.x_dim:
            self.x_region = [(self.x_dim - self.x_limits - 1), self.x_dim]

        if self.y_region[0] < 0:
            self.y_region = [0, self.y_limits - 1]
        elif self.y_region[1] > self.y_dim:
            self.y_region = [self.y_dim - self.y_limits - 1, self.y_dim]

        self.cv_image = self.cv_image[self.y_region[0]:self.y_region[1], self.x_region[0]:self.x_region[1], :]

        self.cv_image = cv2.resize(self.cv_image, (self.x_dim, self.y_dim), interpolation=cv2.INTER_CUBIC)
        image_zoom_filename = os.path.join(directory, "../assets/image_zoomed.png")
        cv2.imwrite(image_zoom_filename, self.cv_image)

        self.picture = tk.PhotoImage(file=image_zoom_filename, format="png")
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        self.image_button = tk.Button(self, image=self.picture, command=lambda: master.switch_frame(PictureSelected))
        self.image_button.pack(side="left", anchor="nw")
        self.image_button.bind("<Button-1>", self.click)
        self.image_button.bind("<Button-1>", self.zoomed_select_button_pub, add="+")

        button_height = 3
        button_width = 8
        self.back_button = tk.Button(self, text="Back", width=button_width, height=button_height, font=large_font,
                                     activebackground="red", bg="red",
                                     command=lambda: master.switch_frame(PictureInitial))
        self.back_button.pack(side="top", anchor="ne", fill="both", padx=5, pady=int(100 * scale_factor))
        self.back_button.bind("<Button-1>", self.zoomed_back_button_pub)

        master.after(update_rate, self.update())

    def update(self):
        """
        Deletes the loading wheel once system is ready, updates image and zooms in on it and loads it. Then  it binds
        image to callback. Just spins wheel if system is waiting

        :return: None
        """
        image_zoom_filename = os.path.join(directory, "../assets/image_zoomed.png")
        self.picture.configure(file=image_zoom_filename, format="png")
        self.after(update_rate, self.update)

    def click(self, event):
        """
        Currently only prints the mouseclick

        :param event: Mouseclick on Image
        :return: None
        """
        image_zoom_filename = os.path.join(directory, "../assets/image_zoomed.png")
        self.cv_image = cv2.imread(image_zoom_filename)
        self.cv_image = cv2.circle(self.cv_image, (event.x, event.y), 10, (0, 255, 0), -1)
        cv2.imwrite(image_zoom_filename, self.cv_image)
        real_x = int(round(zoom_coordinate[0] - (self.x_limits / 2) + event.x / float(self.x_dim) * self.x_limits))
        real_y = int(round(zoom_coordinate[1] - (self.y_limits / 2) + event.y / float(self.y_dim) * self.y_limits))
        if real_x < 0:
            real_x = 0
        elif real_x > self.x_dim:
            real_x = self.x_dim

        if real_y < 0:
            real_y = 0
        elif real_y > self.y_dim:
            real_y = self.y_dim

        global circle_coordinate
        circle_coordinate = (real_x, real_y)

    @staticmethod
    def zoomed_select_button_pub(event):
        log_pub.publish("image_zoomed, image_selected, zoomed_image_button")

    @staticmethod
    def zoomed_back_button_pub(event):
        log_pub.publish("image_zoomed, image_unzoomed, zoomed_back_button")


class PictureSelected(tk.Frame):
    """
    This frame has the image with the selection dot laid over it
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)

        tk.Label(self, text="Please confirm whether this is the correct object", font=small_font).pack()

        image_zoom_filename = os.path.join(directory, "../assets/image_zoomed.png")
        self.picture = tk.PhotoImage(file=image_zoom_filename, format="png")
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        tk.Label(self, image=self.picture).pack(side="left")
        button_height = 3
        button_width = 8
        self.yes_button = tk.Button(self, text="Correct", width=button_width, height=button_height, font=large_font,
                                    activebackground="lime", bg="lime", command=lambda: master.switch_frame(Success))

        self.yes_button.bind("<Button-1>", self.publish_point)
        self.yes_button.bind("<Button-1>", self.selected_correct_button_pub, add="+")

        self.no_button = tk.Button(self, text="Wrong", width=button_width, height=button_height, font=large_font,
                                   activebackground="red", bg="red",
                                   command=lambda: master.switch_frame(PictureInitial))

        self.no_button.bind("<Button-1>", self.selected_incorrect_button_pub)

        self.yes_button.pack(side="top", anchor="ne", padx=5)

        self.no_button.pack(side="top", anchor="ne", padx=5)

        master.after(update_rate, self.update())

    def update(self):
        """
        Populates the image and allows the user to confirm it.

        :return: None
        """

        image_zoom_filename = os.path.join(directory, "../assets/image_zoomed.png")
        self.picture.configure(file=image_zoom_filename, format="png")
        self.after(update_rate, self.update)

    @staticmethod
    def publish_point(event):
        # X, Y format
        array = UInt32MultiArray()

        unzoomed_filename = os.path.join(directory, "../assets/unzoomed.png")
        cv_image = cv2.imread(unzoomed_filename)
        x_dim = cv_image.shape[1]
        downscale_factor = x_dim / 640  # Yields the factor we need to divide circle_coordinates by

        array.data = [int(circle_coordinate[0] / downscale_factor), int(circle_coordinate[1] / downscale_factor)]
        point_pub.publish(array)

    @staticmethod
    def selected_correct_button_pub(event):
        log_pub.publish("image_selected, grasping, selected_correct_button")
        desired_state_pub.publish("pick_object")

    @staticmethod
    def selected_incorrect_button_pub(event):
        log_pub.publish("image_selected, image_unzoomed, selected_incorrect_button")


class Success(tk.Frame):
    """
    Informs the user whether or not their selection was successful depending upon the testers decision
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.label = tk.Label(self, text="Please wait, determining if this was a successful selection", font=small_font)
        self.label.pack()

        self.frame = 0
        loading_wheel_filename = os.path.join(directory, "../assets/loading_wheel.gif")
        self.picture = tk.PhotoImage(file=loading_wheel_filename, format="gif -index {}".format(self.frame))
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        tk.Label(self, image=self.picture).pack(side="left")

        self.button = tk.Button(self, text="Next Grasp", width=12, height=4, font=large_font,
                                command=lambda: master.switch_frame(StartPage))

        master.after(update_rate, self.update())

    def update(self):
        """
        Spins the loading wheel unless the tester has published whether it was a success or not
        """

        if success is None:
            self.wait_spin()

        elif success:
            green_checkmark_filename = os.path.join(directory, "../assets/green_checkmark.png")
            self.picture.configure(file=green_checkmark_filename, format="png")
            self.label.configure(text="Success :)", font=small_font)
            self.button.configure(activebackground="lime", bg="lime")
            self.button.pack(side="right")

        elif not success:
            red_x_filename = os.path.join(directory, "../assets/red_x.png")
            self.picture.configure(file=red_x_filename, format="png")
            self.label.configure(text="Failure :(", font=small_font)
            self.button.configure(activebackground="red", bg="red")
            self.button.pack(side="right")

        self.after(update_rate, self.update)

    def wait_spin(self):
        """
        Spins loading wheel gif

        :return: None
        """

        if self.frame < 16:
            self.frame = self.frame + 1
            loading_wheel_filename = os.path.join(directory, "../assets/loading_wheel.gif")
            self.picture.configure(file=loading_wheel_filename, format="gif -index {}".format(self.frame))
        else:
            self.frame = 0

    @staticmethod
    def next_grasp_button_pub(event):
        # TODO Determine if we want just the success state or the individual result with team
        log_pub.publish("success, begin, next_grasp_button")


def success_cb(data):
    """
    updates the success flag from the /success topic

    :param data: std_msgs.msg Bool
    :return: None
    """
    global success
    success = data.data


def image_cb(data):
    """
    updates the image data that will be displayed. Uses the global variable image_required. When this is high it will
    update the image and pull it low once again

    :param data: sensor_msgs.msg Image
    :return: None
    """
    global image_required
    if image_required:
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            # Hard coded to match depth sensor resolution
            x_dim = int(640 * scale_factor)
            y_dim = int(480 * scale_factor)
            cv_image = cv2.resize(cv_image, (x_dim, y_dim), interpolation=cv2.INTER_CUBIC)

            image_unzoomed = os.path.join(directory, "../assets/unzoomed.png")
            cv2.imwrite(image_unzoomed, cv_image)
            image_required = False

        except CvBridgeError as error:
            print(error)


if __name__ == "__main__":
    rospy.init_node("GUI", anonymous=True)
    rospy.Subscriber("success", Bool, success_cb)
    # TODO Update actual image topic name
    rospy.Subscriber("/rgb_right/image_rect_color", Image, image_cb)
    point_pub = rospy.Publisher("point", UInt32MultiArray, latch=True, queue_size=10)
    log_pub = rospy.Publisher("logging_topic", String, latch=True, queue_size=10)
    desired_state_pub = rospy.Publisher("desired_state", String, latch=True, queue_size=10)
    app = SampleApp()
    app.mainloop()
