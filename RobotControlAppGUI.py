import customtkinter as ctk
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
# window 
window = ctk.CTk()
window.title('customtkinter app')
window.geometry('1355x800')


ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")
# widgets 
string_var = ctk.StringVar(value = 'Control The PlotBot')
label = ctk.CTkLabel(
	window, 
	text = 'Control The PlotBot', 
	#fg = ('blue','red'), 
	#color = ('black','white'),
	corner_radius = 2,
	textvariable = string_var,
 	font =("Arial", 25))
label.place(x=140,y=100)


string_var = ctk.StringVar(value = 'PlotBot Status')
label = ctk.CTkLabel(
	window, 
	text = 'PlotBot Status', 
	#fg = ('blue','red'), 
	#color = ('black','white'),
	corner_radius = 2,
	textvariable = string_var,
 	font =("Arial", 25))
label.place(x=1060,y=100)

string_var = ctk.StringVar(value = 'Drawing Selection')
label = ctk.CTkLabel(
	window, 
	#text = 'PlotBot Status', 
	#fg = ('blue','red'), 
	#color = ('black','white'),
	corner_radius = 2,
	textvariable = string_var,
 	font =("Arial", 25))
label.place(x=610,y=100)


string_var = ctk.StringVar(value = '     Connected    ')
label = ctk.CTkLabel( 
	window, 

	#text = 'PlotBot Status', 
	fg_color = ('blue','green'), 
	text_color = ('black','white'),
	corner_radius = 6,
	textvariable = string_var,
 	font =("Arial", 30))
label.place(x=1030,y=150)
print(dir(ctk.CTkLabel(window)))

string_var = ctk.StringVar(value = 'Drawing Sended')
label = ctk.CTkLabel(
	window, 

	#text = 'PlotBot Status', 
	fg_color = ('blue','white'), 
	text_color = ('black','black'),
	corner_radius = 6,
	textvariable = string_var,
 	font =("Arial", 30))
label.place(x=1030,y=200)


string_var = ctk.StringVar(value = '    In Progress    ')
label = ctk.CTkLabel(
	window, 

	#text = 'PlotBot Status', 
	fg_color = ('blue','white'), 
	text_color = ('black','black'),
	corner_radius = 6,
	textvariable = string_var,
 	font =("Arial", 30))
label.place(x=1030,y=250)

string_var = ctk.StringVar(value = '    Completed     ')
label = ctk.CTkLabel(
	window, 

	#text = 'PlotBot Status', 
	fg_color = ('blue','white'), 
	text_color = ('black','black'),
	corner_radius = 6,
	textvariable = string_var,
 	font =("Arial", 30))
label.place(x=1030,y=300)
print(dir(ctk.CTkLabel(window)))

def buton1():
	pass


def select(event):
	global imgHopstıch
	global imgFootballPitch

	if event == "Hopstoch":
		img23 = ImageTk.PhotoImage(Image.open("libs\images\Hopstoch.jpg").resize((400,400),Image.ANTIALIAS)),
		labela.configure(image=img23)
	if event == "Football Pitch":
		img23 = ImageTk.PhotoImage(Image.open("libs\images\FootballPitch.jpg").resize((400,400),Image.ANTIALIAS)),
		labela.configure(image=img23)
	

Drop = ctk.CTkOptionMenu(window,command=select,	values=["Hopstoch","Football Pitch"],
    width=440,
    font=("arial",25),
    height=70).place(x=500,y=150)


img = ImageTk.PhotoImage(Image.open("libs\images\Hopstoch.jpg"),Image.ANTIALIAS)

labela = ctk.CTkLabel(
    
	window, 
 text="",
	image=img,
	#bg_color=("white"),
	#text = 'PlotBot Status', 
	fg_color = ('white','white'), 
	text_color = ('white'),
	corner_radius = 10,
 width=440,
 height=390
	#textvariable = string_var,
 	#font =("Arial", 30))
)
labela.place(x=500,y=240)

SendOpretionButton = ctk.CTkButton(
	master=window,
 	text="SEND DRAW",
	
	width=440,
	height=102,
	 #compound="left",
 	fg_color='#FFFFF0',
  text_color="#0A0A0A",
  font=("Arial",36),
	hover_color='#303030',
 command=buton1).place(x=500,y=650)

ForwardArrayImage = ImageTk.PhotoImage(Image.open("libs\images\FowardArray.png").resize((100,100),Image.ANTIALIAS))
ForwardButton = ctk.CTkButton(
	master=window,
 	text="",
	image=ForwardArrayImage,
	width=102,
	height=102,
	 #compound="left",
 	fg_color='#000',
	hover_color='#303030',
 command=buton1).place(x=190,y=150)

BackwardArrayImage = ImageTk.PhotoImage(Image.open("libs\images\BackwardArray.png").resize((100,100),Image.ANTIALIAS))
BackwardButton = ctk.CTkButton(
	master=window,
 	text="",
	image=BackwardArrayImage,
	width=102,
	height=102,
	 #compound="left",
 	fg_color='#000',
	hover_color='#303030',
 command=buton1).place(x=190,y=270)

LeftArrayImage = ImageTk.PhotoImage(Image.open("libs\images\LeftArray.png").resize((100,100),Image.ANTIALIAS))
BackwardButton = ctk.CTkButton(
	master=window,
 	text="",
	image=LeftArrayImage,
	width=102,
	height=102,
	 #compound="left",
 	fg_color='#000',
	hover_color='#303030',
 command=buton1).place(x=70,y=270)


RightArrayImage = ImageTk.PhotoImage(Image.open("libs\images\RightArray.png").resize((100,100),Image.ANTIALIAS))
BackwardButton = ctk.CTkButton(
	master=window,
 	text="",
	image=RightArrayImage,
	width=102,
	height=102,
	 #compound="left",
 	fg_color='#000',
	hover_color='#303030',
 command=buton1).place(x=310,y=270)



StartOpretionButton = ctk.CTkButton(
	master=window,
 	text="START",
	
	width=352,
	height=102,
	 #compound="left",
 	fg_color='#FFFFF0',
  text_color="#0A0A0A",
  font=("Arial",36),
	hover_color='#303030',
 command=buton1).place(x=70,y=400)

StopOpretionButton = ctk.CTkButton(
	master=window,
 	text="STOP",
	
	width=352,
	height=102,
	 #compound="left",
 	fg_color='#FFFFF0',
  text_color="#0A0A0A",
  font=("Arial",36),
	hover_color='#303030',
 command=buton1).place(x=70,y=530)


# run
window.mainloop()