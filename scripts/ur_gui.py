try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import rospy
from std_msgs.msg import Bool

gui_wait = None
success = None
update_rate = 100  # In milliseconds


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
        self.button = tk.Button(self, text="Begin", width=25, height=10,
                                command=lambda: master.switch_frame(PictureSelect))
        self.button.bind("<Button-1>", self.clear_globals)
        self.button.pack(fill="both", expand=True, padx=100, pady=100)

        master.after(update_rate, self.update())

    def update(self):
        """No necessary updates in this frame"""
        self.after(update_rate, self.update)

    @staticmethod
    def clear_globals(event):
        """
        resets the gui_wait and success variables to None
        :param event: Start Page Button Push
        :return: None
        """
        global gui_wait
        gui_wait = None
        global success
        success = None


class PictureSelect(tk.Frame):
    """
    This frame loads the initial unzoomed image taken from the camera, receives a user click, and then publishes over
    ROS
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)

        self.frame = 0

        self.label = tk.Label(self, text="Please click the object on the touchscreen").pack(side="top", fill="x",
                                                                                            pady=10)

        self.picture = tk.PhotoImage(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
        self.image_label = tk.Label(self, image=self.picture)
        self.image_label.pack()
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        self.image_button = tk.Button(self, image=self.picture, command=lambda: master.switch_frame(PictureSelected))

        global gui_wait
        gui_wait = True

        master.after(update_rate, self.update())

    def update(self):
        """
        Deletes the loading wheel once system is ready, updates image and loads it. Then  it binds image to callback.
        Just spins wheel if system is waiting

        :return: None
        """
        if not gui_wait:
            self.image_label.destroy()
            self.picture.configure(file="rgb_right.png", format="png")
            self.image_button.pack(side="left")
            self.image_button.bind("<Button-1>", self.click)
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
            self.picture.configure(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
        else:
            self.frame = 0

    @staticmethod
    def click(event):
        """
        Currently only prints the mouseclick
        #TODO Implement ROS Publisher and tap to zoom

        :param event: Mouseclick on Image
        :return: None
        """
        print("Event done")
        print("Mouse position: {} {}".format(event.x, event.y))
        return


class PictureSelected(tk.Frame):
    """
    This frame has the image with the selection dot layed over it
    #TODO Implement selection Dot and zoom
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)

        tk.Label(self, text="Please confirm whether this is the correct object").pack()

        self.frame = 0
        self.picture = tk.PhotoImage(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        tk.Label(self, image=self.picture).pack(side="left")

        self.yes_button = tk.Button(self, text="Correct", width=25, height=13,
                                    command=lambda: master.switch_frame(Success))

        self.no_button = tk.Button(self, text="Wrong", width=25, height=13,
                                   command=lambda: master.switch_frame(PictureSelect))

        global gui_wait
        gui_wait = True
        print("gui_wait value:{}".format(gui_wait))

        master.after(update_rate, self.update())

    def update(self):
        """
        Loads the screen until gui_wait is pulled low, then it populates the image and allows the user to confirm it.

        :return: None
        """
        global gui_wait
        if not gui_wait:
            self.yes_button.pack(side="top", anchor="ne")
            self.no_button.pack(side="top", anchor="ne")
            self.picture.configure(file="rgb_right.png", format="png")
        elif gui_wait is None:
            print("Error gui_wait is of type None")
        else:
            if self.frame < 16:
                self.frame = self.frame + 1
                self.picture.configure(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
            else:
                self.frame = 0
        self.after(update_rate, self.update)


class Success(tk.Frame):
    """
    Informs the user whether or not their selection was successful depending upon the testers decision
    """

    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.label = tk.Label(self, text="Please wait, determining if this was a successful selection")
        self.label.pack()

        self.frame = 0
        self.picture = tk.PhotoImage(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
        tk.Frame.photo = self.picture  # Needed to prevent garbage collector
        tk.Label(self, image=self.picture).pack(side="top")

        self.button = tk.Button(self, text="Next Grasp", width=25, height=10,
                                command=lambda: master.switch_frame(StartPage))

        global gui_wait
        gui_wait = True

        print("gui_wait value:{}".format(gui_wait))
        print("success value:{}".format(success))
        master.after(update_rate, self.update())

    def update(self):
        """
        Spins the laoding wheel unless the tester has both published a success message, and the system has pulled
        gui_wait low
        #TODO Get better clip art

        :return: none
        """
        global gui_wait

        if success is None:
            self.wait_spin()

        elif gui_wait is None:
            print("Error None value received for gui_wait")

        elif (not gui_wait) and success:
            self.picture.configure(file="green_checkmark.png", format="png")
            self.label.configure(text="Success :)")
            self.button.pack(side="bottom")

        elif (not gui_wait) and (not success):
            self.picture.configure(file="red_x.png", format="png")
            self.label.configure(text="Failure :(")
            self.button.pack(side="bottom")

        else:
            self.wait_spin()

        self.after(update_rate, self.update)

    def wait_spin(self):
        """
        Spins loading wheel gif

        :return: None
        """
        if self.frame < 16:
            self.frame = self.frame + 1
            self.picture.configure(file="loading_wheel.gif", format="gif -index {}".format(self.frame))
        else:
            self.frame = 0


def gui_wait_cb(data):
    """
    updates the gui_wait flag from the /gui_wait topic

    :param data: std_msgs.msg Bool
    :return: None
    """
    global gui_wait
    gui_wait = data.data


def success_cb(data):
    """
    updates the success flag from the /success topic

    :param data: std_msgs.msg Bool
    :return: None
    """
    global success
    success = data.data


if __name__ == "__main__":
    rospy.init_node("Listener", anonymous=True)
    rospy.Subscriber("gui_wait", Bool, gui_wait_cb)
    rospy.Subscriber("success", Bool, success_cb)
    app = SampleApp()
    # app.wm_geometry("600x600")
    app.mainloop()