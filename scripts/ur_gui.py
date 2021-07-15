try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

import rospy
from std_msgs.msg import Bool

#TODO Flag testing only
flag = None

class SampleApp(tk.Tk):
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
        #TODO Flag here for testing only
        print("flag_value:{}".format(flag))


class StartPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        tk.Button(self, text="Begin", width=25, height=10,
                  command=lambda: master.switch_frame(PictureSelect)).pack(fill="both", expand=True, padx=100, pady=100)



class PictureSelect(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.button_pushed = False

        tk.Label(self, text="Please click the object on the touchscreen").pack(side="top", fill="x", pady=10)
        tk.Button(self, text="Return to start page",
                  command=lambda: master.switch_frame(StartPage)).pack()

        picture = tk.PhotoImage(file="rgb_right.png")
        tk.Frame.photo = picture  # Needed to prevent garbage collector
        image_button = tk.Button(self, image=picture, command=lambda: master.switch_frame(PictureSelected))
        image_button.bind("<Button-1>", self.click)
        image_button.pack(side="left")

    def click(self, event):
        print("Event done")
        print("Mouse position: {} {}".format(event.x, event.y))
        return


class PictureSelected(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)

        tk.Label(self, text="Please confirm whether this is the correct object").pack()

        picture = tk.PhotoImage(file="rgb_right.png")
        tk.Frame.photo = picture  # Needed to prevent garbage collector
        tk.Label(self, image=picture).pack(side="left")

        tk.Button(self, text="Correct", width=25, height=13,
                  command=lambda: master.switch_frame(ConfirmSuccess)).pack(side="top", anchor="ne")

        tk.Button(self, text="Wrong", width=25, height=13,
                  command=lambda: master.switch_frame(PictureSelect)).pack(side="top", anchor="ne")


class ConfirmSuccess(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        tk.Label(self, text="Please wait, system is processing").pack()
        #TODO This is here for testing only
        global flag
        flag = None

class Success(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        tk.Label(self, text="Success, object selected properly").pack()

class Failure(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        tk.Label(self, text="Failure, object missed").pack()

def flag_cb(data):
    global flag
    flag = data.data
    #TODO Flag here for testing
    print("Flag Value:{}".format(flag))


if __name__ == "__main__":
    rospy.init_node("Listener", anonymous=True)
    #Flag testing only
    rospy.Subscriber("flag", Bool, flag_cb)
    app = SampleApp()
    # app.configure(bg="DarkSlateGray")
    # app.wm_geometry("600x600")
    app.mainloop()
