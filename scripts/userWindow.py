#!/usr/bin/env python
import tkinter as tk
import rospy
from std_msgs.msg import String
import signal

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('user_response', String, queue_size=10)

# Variables for creating the window with question for the user.
window = tk.Tk()
question = tk.Label(window)

#--------- FUNCTION ---------#
# Function that generate the window.
def createWindow():
    # Define width and hight of the window.
    width = 600 # Width 
    height = 150 # Height
    
    screen_width = window.winfo_screenwidth()  # Width of the screen
    screen_height = window.winfo_screenheight() # Height of the screen
    
    # Calculate starting X and Y coordinates for the window.
    x = (screen_width/2) - (width/2)
    y = (screen_height/2) - (height/2)
    
    # Define the window aspect.
    # Position the window in the center of the screen.
    window.geometry('%dx%d+%d+%d' % (width, height, x, y))
    window.title("No solution found")
    window.resizable(False, False)
    window.configure(background="white")

    # Set the question aspect.
    question.config(text = "Have you found a solution?", # This is a generic question. It will be modified later.
                    background = "white",
                    font = ("Helvetica", 15),
                    wraplength= 300)
    question.grid(row = 0, column = 0, columnspan = 2, padx = 160, pady = 20)

    # Set YES Button aspect and function to call when clicked.
    yesButton = tk.Button(window, text = "Yes", width = 10, background = "white",
                            highlightthickness=2, highlightcolor="black", highlightbackground="black", 
                            font=("Helvetica", 15), command= yesAction)
    yesButton.grid(row=1, column=0)
    
    # Set NO Button aspect and function to call when clicked.
    noButton = tk.Button(window, text = "No", width = 10, background = "white",
                            highlightthickness=2, highlightcolor="black", highlightbackground="black", 
                            font=("Helvetica", 15), command = noAction)
    noButton.grid(row=1, column=1)

    # Set the function to call when the window is closed with the close button of the title bar.
    window.protocol("WM_DELETE_WINDOW", on_closing)

# Function that publishes when the YES button is clicked.
def yesAction():
    pub.publish("Y")
    # Hide the window.
    window.withdraw()

# Function that publishes when the NO button is clicked.
def noAction():
    pub.publish("N")
    window.withdraw()  

# Function that publishes when the the close button of the title bar is clicked.
def on_closing():
    pub.publish("N")
    # Hide the window.
    window.withdraw()   

# Function called when no solution is found for an error in the production line.
# This function sets the question based on the received message.
def showUserWindow(data):
    # Retrive the information of the error from the message and set the question.
    list = data.data.split(";")
    errorType = list[0]
    stationError = list[1]
    textQuestion = "Error:%s at station %s. Have you found a solution?" % (errorType, stationError)
    question.config(text= textQuestion)

    # Show the window.
    window.deiconify()

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node and the FlexBE behavior "Production Line".
def listener():
    rospy.init_node('user_window', anonymous=True)
    rospy.Subscriber('ask_question', String, showUserWindow)

# Function used to destroy the window when the user executes <Control-C>.
def handler(event):
    window.destroy()

# Function needed for check when the user executes <Control-C>.
def check():
    window.after(500, check)

if __name__ == "__main__":
    # Initialize the ROS node and start the subscribers.
    listener()

    createWindow()

    # Hide the window 
    window.withdraw()

    # Give the oppurtunity to close the window with <Control-C> command.
    signal.signal(signal.SIGINT, lambda x,y : handler(None))
    window.after(500, check)
    window.bind_all('<Control-c>', handler)

    # Execute the mainloop to process the window information.
    # Note: In the listener function, rospy.spin() is not present.
    # The same goal is now achieved with the window.mainloop().
    window.mainloop()