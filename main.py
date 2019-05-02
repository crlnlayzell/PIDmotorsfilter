import motor as mo
import time as t
import pdb

mo.get_volt()

count = 0
''
while(count < 4):
    (xp, yp) = mo.move(0,0,500,0)
    print("xp %d, yp %d" %(xp, yp))
    (xp, yp) = mo.move(xp,yp,500,500)   
    print("xp %d, yp %d" %(xp, yp))
    (xp, yp) = mo.move(xp, yp,0,500)
    print("xp %d, yp %d" %(xp, yp))
    (xp, yp) = mo.move(xp, yp, 0,0)
    count +=1
''

#mo.move(0,0,500,0)
#(xp, yp) = mo.move(0,0,1000,0)
#(xp, yp) = mo.move(0,0,-600,200)

#(xp, yp)=mo.move(0,0, 1000, 0)


#diamond
'''
count=0
while(count < 5):
    (xp, yp) = mo.move(0,0,353, 353)
    (xp, yp) = mo.move(xp,yp,706, 0)
    (xp, yp) = mo.move(xp,yp,353,-353)
    (xp, yp) = mo.move(xp, yp, 0, 0)
    count+=1
'''

print("xp %d, yp %d" %(xp, yp))
