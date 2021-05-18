import math
import matplotlib
import matplotlib.pyplot
import numpy as np
from threading import Thread
from time import sleep
from dft import DFT

# Object for creating all of the epicyles
class Epicycle():
    def __init__(self, x,y, fourier, rotation, timer, color="red"):
        self.timer = timer
        self.fourier = fourier
        self.cycles = len(fourier.X)
        self.circles = []
        self.x = x
        self.y = y
        # Array of points for creating each circle in the perimeter of the previous
        # Last value is the tip and it's used to draw the figure
        self.xs = [self.x]
        self.ys = [self.y]
        for i in range(self.cycles):
            self.prevx = self.x
            self.prevy = self.y
            # Store values of DFT for readability
            self.freq = self.fourier.X[i]["freq"]
            self.amplitude = self.fourier.X[i]["amp"]
            self.phase = self.fourier.X[i]["phase"]
            self.x += self.amplitude*math.cos(self.freq * self.timer + self.phase + rotation)
            self.y += self.amplitude*math.sin(self.freq * self.timer + self.phase + rotation)
            self.circle = matplotlib.patches.Circle((self.prevx,self.prevy), 
                                                    self.amplitude, fill=False, color=color)
            # Store values of xs and ys for creating the arrows (To do)
            self.xs.append(self.x)
            self.ys.append(self.y)
            # Store array of circles for drawing them on plot
            self.circles.append(self.circle)


class Plotter():
    timer = 0
    def __init__(self, fig, ax, signalX, signalY):
        # Stop the thread when closed
        fig.canvas.mpl_connect('close_event', self.stop)
        self.fig = fig
        self.ax = ax
        # Create arrays for path visualization. Will store final value of xs and ys of epicycles
        self.points_x = []
        self.points_y = []
        # List for creation of lines in order to update data
        self.dummy_list = np.zeros(len(signalX))
        self.line, = self.ax.plot(self.dummy_list, self.dummy_list)
        # Perform discrete fourier transform on signals
        self.fourierX = DFT(signalX)    
        self.fourierY = DFT(signalY)

        
    def update_plot(self):
        while self.started:
            try:
                # Create epicycle for control of x and y components
                ep_y = Epicycle(-30, 0, self.fourierY, math.pi/2, self.timer, 
                                color="green")
                ep_x = Epicycle(0, 30, self.fourierX, 0, self.timer)
                # Add circles to plot
                for i in range(ep_y.cycles):
                    self.ax.add_patch(ep_x.circles[i])
                    self.ax.add_patch(ep_y.circles[i])
                
                # Add to the list of points the tip of the last circle
                self.points_x.append(ep_x.xs[-1])
                self.points_y.append(ep_y.ys[-1])
                # Update data to plot
                self.line.set_xdata(self.points_x)
                self.line.set_ydata(self.points_y)
                # Update plot
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                # Advance time in simulation by dt (as a function of the resolution 
                # of the plot)
                self.timer += math.tau/ep_x.cycles
                # self.ax.autoscale()
                sleep(0.05)
                # Clear circles for next iteration
                for i in ep_x.circles:
                    i.remove()
                for i in ep_y.circles:
                    i.remove()
                # Clear lists when a complete cycle is done (time> 2*pi as we're
                # dealing with fourier and cycles complete after 2pi duh)
                if self.timer > math.tau:
                    self.points_x = []
                    self.points_y = []
                    self.timer = 0

            except KeyboardInterrupt:
                break
    
    # Start thread creation
    def start(self):
        self.started = True
        self.thread = Thread(target=self.update_plot)
        self.thread.daemon = True
        self.thread.start()

    def stop(self, close_event):
        self.started = False


if __name__ == "__main__":
    signalx = []
    signaly = []
    resolution = 50
    # Create signal for x and y coordinates
    # Do this without parametric curves or from images?
    for i in range(resolution):
        t = i*math.tau/resolution
        x = 16*math.sin(t)**3
        y = 13*math.cos(t) - 5*math.cos(2*t) - 2*math.cos(3*t) - math.cos(4*t)
        signalx.append(x)
        signaly.append(y)

    # for i in range(resolution):
    #     angle = i*math.tau/resolution
    #     signalx.append(8*math.cos(angle))
    #     signaly.append(8*math.sin(angle))
        

    # Establish the backend
    matplotlib.use("Qt5Agg")
    # Creates figures and axes that holds the plot
    fig, ax = matplotlib.pyplot.subplots(1,1, figsize=(9,9))
    # Create plotter object that does the job
    ep = Plotter(fig, ax, signalx, signaly)
    # Start thread
    ep.start()
    
    # s = 30
    # Set limits of the axes
    matplotlib.pyplot.xlim([-50,20])
    matplotlib.pyplot.ylim([-20,50])
    # Wait a bit until everything is created
    # Remove this and you shall perish
    sleep(1)
    # Show the window
    matplotlib.pyplot.show()
    
    