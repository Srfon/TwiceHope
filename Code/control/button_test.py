from gpiozero import Button

btn1 = Button(22)
btn3 = Button(27)
btn4 = Button(17)
  
def pressed(n):
    print(f"Button {n} pressed")

btn1.when_pressed = lambda: pressed(1)
btn3.when_pressed = lambda: pressed(3)
btn4.when_pressed = lambda: pressed(4)

while True:
    pass
