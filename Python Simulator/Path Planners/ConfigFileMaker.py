import Tkinter as tk
import sys

# Set number of rows and columns
ROWS = 20
COLS = 20

f=open("one.config","w")
'''
File writer in original config file format
'''
f.write('height {0} \n'.format(ROWS))
f.write('width {0} \n\n\n'.format(COLS))
f.write('R 50\n\n\n')
f.write('baseX 0\n')
f.write('baseY 0\n\n\n')
f.write('obstacles\n')


# Create a grid of None to store the references to the tiles
tiles = [[None for i in range(COLS)] for j in range(ROWS)]
a = [[0]*COLS for i in range(ROWS)]
def callback(event):
    # Get rectangle diameters
    col_width = c.winfo_width()/COLS
    row_height = c.winfo_height()/ROWS
    # Calculate column and row number
    col = int(event.x//col_width)
    row = int(event.y//row_height)
    # print(row,col)
    # If the tile is not filled, create a rectangle
    if not tiles[row][col]:
        tiles[row][col] = c.create_rectangle(col*col_width, row*row_height, (col+1)*col_width, (row+1)*row_height, fill="black")
        a[row][col] = 1
        f.write(str(row)+" "+str(col)+"\n")
    # If the tile is filled, delete the rectangle and clear the reference
    else:
        c.delete(tiles[row][col])
        tiles[row][col] = None

# Create the window, a canvas and the mouse click event binding
root = tk.Tk()
c = tk.Canvas(root, width=800, height=800, borderwidth=0, background='white')
c.pack()
c.bind("<Button-1>", callback)

def ok():
    print("Config file prepared")
    
def exit():
    for i in a:
        print(' '.join(str(j) for j in i))
    sys.exit()

obstacles = tk.Button(root,text="Set",command=ok)
quit = tk.Button(root,text="Exit",command=exit)
obstacles.pack() ; quit.pack()
root.mainloop()
f.close()
