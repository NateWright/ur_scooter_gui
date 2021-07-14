try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk


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

class StartPage(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        tk.Button(self, text="Begin", width=25, height=10,
                  command=lambda: master.switch_frame(PageOne)).pack(fill="both", expand=True, padx=100, pady=100)

class PageOne(tk.Frame):
    def __init__(self, master):
        tk.Frame.__init__(self, master)
        self.button_pushed = False

        tk.Label(self, text="Please click the object on the touchscreen").pack(side="top", fill="x", pady=10)
        tk.Button(self, text="Return to start page",
                  command=lambda: master.switch_frame(StartPage)).pack()

        picture = tk.PhotoImage(file="rgb_right.png")
        tk.Frame.photo = picture    #Needed to prevent garbage collector
        image_label = tk.Label(self, image=picture)
        image_label.bind("<Button-1>", self.click)
        image_label.pack(side="left")

    def click(self, event):
        #print(arg)
        print("Mouse position: {} {}".format(event.x, event.y))
        self.button_pushed = True
        return


class PageTwo(tk.Frame):
    def __init__(self, master):
        tk.Label(self, text="Please confirm whether this is the correct object").pack(side="top", fill="x", pady=10)
        tk.Button(self, text="Return to start page",
                  command=lambda: master.switch_frame(StartPage)).pack()
        tk.Button(self, text="True",
                  command=lambda: master.switch_frame(PageOne)).pack()
        tk.Button(self, text="False",
                  command=lambda: master.switch_frame(StartPage)).pack()

        picture = tk.PhotoImage(file="rgb_right.png")
        tk.Frame.photo = picture    #Needed to prevent garbage collector
        image_label = tk.Label(self, image=picture)
        image_label.bind("<Button-1>", motion)
        image_label.pack(side="left")

"""
def motion(event):
    print("Mouse position: {} {}".format(event.x, event.y))
    return
"""

if __name__ == "__main__":
    app = SampleApp()
    #app.configure(bg="DarkSlateGray")
    #app.wm_geometry("600x600")
    app.mainloop()
