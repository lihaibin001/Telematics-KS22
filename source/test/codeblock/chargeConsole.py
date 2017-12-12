from tkinter import *
import tkinter.messagebox
import serial


def charge_request_callback():
    tkinter.messagebox.showinfo("Prompt", "Gun has plugged in! ")
# Open serial
TUV = serial.Serial('com3', 9600)
# Write to serial
n = TUV.write(b"12345")
# listen the serial
STR = TUV.read(n)
print(STR)
root = tkinter.Tk()
charge_request_button = tkinter.Button(root, text="user charge", command=charge_request_callback)
charge_request_button.pack()
# enter main loop
root.mainloop()
