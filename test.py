
import numpy as np



class Cat:
    def __init__(self, name, mouth, eyes): # constructor
        self.name = name
        self.tail = None
        self.eyes = None
        self.mouth = None

    def poop(self):
        print(f"Hi i am a cat named {self.name}, and i am pooping")
        pass

    def miau(self):

        pass
    def kiss(self):
        pass

listI = range(5)
listJ = listI[::-1]

for (i,j) in zip(listI, listJ):
    print(i, j)


myList = listI[::-3]

for thingy in myList:
    print(thingy)