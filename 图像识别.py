import sensor
import math
import utime
import colorthre as ct
#colorthre_blobs = [ \
#        (0,90,-128,-4,0,128), \
#        (30,80,42,128,52,8), \
#        (0,100,-128,35,-128,-10), \
#        (0,60) \
#    ]
from machine import UART
from fpioa_manager import *
import os, Maix, lcd, image
from Maix import FPIOA, GPIO

#init screen
lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)

finding_rect = (130,100,120,100)
finding_size = (0,0)

maxRectSize = 200
minRectSize = 30

laserPos = (160,120)

lastProcessLaser = 0

fm.register(10, fm.fpioa.GPIO0)
fm.register(11, fm.fpioa.GPIO1)
hc_trig = GPIO(GPIO.GPIO0,GPIO.OUT)
hc_echo = GPIO(GPIO.GPIO1,GPIO.IN)

fm.register(14, fm.fpioa.GPIO2)
sound = GPIO(GPIO.GPIO2,GPIO.OUT)
sound.value(1)
utime.sleep_ms(500)
sound.value(0)

fm.register(9,fm.fpioa.UART2_TX)
fm.register(15,fm.fpioa.UART2_RX)
uart = UART(UART.UART2,9600)

fm.register(17, fm.fpioa.GPIO3)
button = GPIO(GPIO.GPIO3,GPIO.IN,GPIO.PULL_UP)

hc_trig.value(1)
utime.sleep_us(15)
hc_trig.value(0)

def getLenByHCPX(hc,px):
    return (hc*px)/320.6395#342.0155

def getCross_point(line1, line2):
    a1 = line1.y2() - line1.y1()
    b1 = line1.x1() - line1.x2()
    c1 = line1.x2()*line1.y1() - line1.x1()*line1.y2()

    a2 = line2.y2() - line2.y1()
    b2 = line2.x1() - line2.x2()
    c2 = line2.x2() * line2.y1() - line2.x1()*line2.y2()

    try:
        if (a1 * b2 - a2 * b1) != 0 and (a2 * b1 - a1 * b2) != 0:
            cross_x = int((b1*c2-b2*c1)/(a1*b2-a2*b1))
            cross_y = int((c1*a2-c2*a1)/(a1*b2-a2*b1))
            return (cross_x, cross_y)
    except:
        return (-1, -1)
    return (-1, -1)


def getVtxs(lines):
    vtxs = []
    line_num = len(lines)
    for i in range(line_num -1):
        for j in range(i, line_num):
            vtx = getCross_point(lines[i], lines[j])
            if vtx[0]>0 and vtx[1]>0 :
                vtxs.append(vtx)
    return vtxs

def getMinVtxLen(vtxs):
    vtx_num = len(vtxs)
    lenmin = 99999999
    for i in range(vtx_num -1):
        for j in range(i, vtx_num):
            try:
                lx = vtxs[i][0] - vtxs[j][0]
                ly = vtxs[i][1] - vtxs[j][1]
                plx = lx*lx
                ply = ly*ly
                llen = math.sqrt(plx+ply)
                if llen<lenmin and llen>1 :
                    lenmin = llen
            except:
                pass
    return lenmin

def getShapeBy2Line(lines):
    line1 = lines[0]
    line2 = lines[1]
    v1 = (line1.x1()-line1.x2()  ,  line1.y1()-line1.y2())
    v2 = (line2.x1()-line2.x2()  ,  line2.y1()-line2.y2())
    l1 = math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])
    l2 = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1])
    #print(l1,l2)
    if l1==0 or l2==0:
        return (0,0,0)
    try:
        v1 = (v1[0]/l1 , v1[1]/l1)
        v2 = (v2[0]/l2 , v2[1]/l2)
        angle = math.degrees(math.acos(math.fabs(v1[0]*v2[0]+v1[1]*v2[1])))
        if angle>50 and angle<70 :
            return (2,max(l1,l2),angle)
        elif angle>80 and angle<100 :
            return (3,max(l1,l2),angle)
    except:
        return (0,0,0)
    return (0,0,angle)

def blob_merge_cb(A,B):
    if A.w()>maxRectSize and B.h()>maxRectSize :
        return False
    if B.w()>maxRectSize and B.h()>maxRectSize :
        return False
    return True

process_startTime = utime.ticks_ms()

cam_center = (160,120)

lock_ignore_3D = True

while True:
    lineNum = 0
    cam_move = 0
    soccer_1 = None
    soccer_2 = None
    bask = None
    fetched_color = None
    if button.value()==0 :

        button_startTime = utime.ticks_ms()

        #wait for release button
        while button.value()==0 :
            pass

        button_delta_time = math.fabs(utime.ticks_ms() - button_startTime)
        if button_delta_time < 2000 :
            lock_ignore_3D = not lock_ignore_3D
        else:
            ntm = utime.ticks_ms()
            sensor.set_auto_gain(False)
            sensor.skip_frames(50) # Let new settings take affect.
            sensor.set_auto_exposure(False, 1400)
            sensor.set_auto_whitebal(False)

            while math.fabs(utime.ticks_ms()-ntm)<10000:
                img=sensor.snapshot()
                laserBlobs = img.find_blobs([(30,100)],x_stride=1,y_stride=1)
                laserPos = (160,120)
                if laserBlobs:
                    for b in laserBlobs:
                        if b.w()<30 and b.h()<30 :
                            laserPos = (b.x() , b.y())
                if laserPos!=None :
                    img.draw_circle(laserPos[0],laserPos[1],2,color=(0,0,255))
                img.draw_string(0, 0, "finding laser "+str(10-int(math.fabs(utime.ticks_ms()-ntm)/1000)), scale=2 , color = (128,255,255))
                lcd.display(img)

            sensor.set_auto_gain(True)
            sensor.skip_frames(50) # Let new settings take affect.
            sensor.set_auto_exposure(True)
            sensor.set_auto_whitebal(True)

    angle = None
    #print('hc_start')
    hc_trig.value(1)
    utime.sleep_us(15)
    hc_trig.value(0)
    hc_startTime = utime.ticks_us()
    try:
        while True:
            if hc_echo.value() == 1 :
                break
            if math.fabs(utime.ticks_us()-hc_startTime)>40000 :
                break
        hc_startTime = utime.ticks_us()
        while True:
            if hc_echo.value() == 0 :
                break
            if math.fabs(utime.ticks_us()-hc_startTime)>40000 :
                break
        hc_delta = utime.ticks_us()-hc_startTime
        hc_len = hc_delta*0.00005*340
    except:
        hc_len = 0

    img=sensor.snapshot()
    if laserPos!=None :
        fetched_color = img.get_pixel(laserPos[0] , laserPos[1])
    blobs = img.find_blobs(ct.colorthre_blobs,x_stride=16,y_stride=16,area_threshold=16,margin=8,merge=True,merge_cb=blob_merge_cb)

    haveLaser = False
    nowLen = 999999
    if blobs:
        for b in blobs:
            if b.w()>minRectSize and b.h()>minRectSize and b.w()<maxRectSize and b.h()<maxRectSize :

                if haveLaser == False :
                    finding_rect = (b.x()-3 , b.y()-3 , b.w()+6 , b.h()+6)
                    finding_size = (b.w(),b.h())
                    break

                #find laser
                if laserPos!=None and laserPos[0]>b.x() and laserPos[1]>b.y() and laserPos[0]<b.x()+b.w() and laserPos[1]<b.y()+b.h() :
                    haveLaser = True
                    break

                if laserPos!=None :
                    bcenter = (b.x()+b.w()*0.5 , b.y()+b.h()*0.5)
                    blen2 = (bcenter[0]-laserPos[0] , bcenter[1]-laserPos[1])
                    blen = math.sqrt(blen2[0]*blen2[0] + blen2[1]*blen2[1])
                    if blen<nowLen :
                        nowLen = blen
                        finding_rect = (b.x()-3 , b.y()-3 , b.w()+6 , b.h()+6)
                        finding_size = (b.w(),b.h())

        center = (finding_rect[0]+finding_rect[2]/2 , finding_rect[1]+finding_rect[3]/2)
        delta_cam = (center[0]-cam_center[0] , center[1]-cam_center[1])
        if math.fabs(delta_cam[0])>5 :
            if delta_cam[0]>0 :
                cam_move = 4
            else:
                cam_move = 3
        elif math.fabs(delta_cam[1])>5 :
            if delta_cam[1]>0 :
                cam_move = 2
            else:
                cam_move = 1

    if not lock_ignore_3D:
        finding_rect = (130 , 95 , 60 , 50)
        finding_size = (60,50)
        cam_move = 0

    #type of shape
    shapeType = 0
    cir_r = 0
    minlen = 0

    # find cir
    ballType = 0
    cir = img.find_circles(threshold=800,roi=finding_rect)
    #if not cir:
    #    cir = img.find_circles(threshold=800,roi=(130,95,60,50))
    if cir :
        avaiCir = []
        for c in cir :
            img.draw_circle(c.x(),c.y(),c.r(),color=(255,255,0))
            r2 = c.r() * 2
            if r2 > minRectSize and r2 < maxRectSize :
                avaiCir.append(c)

        if len(avaiCir)==1 :
            #put data to buffer
            shapeType = 1
            cir_r = avaiCir[0].r()

        #find ball
        soccer_1 = img.find_blobs([(0,30,-5,5,-5,5)],x_stride=1,y_stride=1,roi=finding_rect)
        soccer_2 = img.find_blobs([(50,100,-5,5,-5,5)],x_stride=1,y_stride=1,roi=finding_rect)
        bask = img.find_blobs([(30,65,3,5,7,40)],x_stride=1,y_stride=1,roi=finding_rect)
        volley_1 = img.find_blobs([(88,95,0,-44,93,48)],x_stride=1,y_stride=1,roi=finding_rect)
        volley_2 = img.find_blobs([(0,80,-128,35,-128,-18)],x_stride=1,y_stride=1,roi=finding_rect)

        if volley_1 and volley_2:
            ballType = 3
        elif bask :
            ballType = 2
        elif soccer_1 and soccer_2 :
            ballType = 1

    if shapeType!=1 :

        # find lines
        lines = img.find_lines(threshold = 400, theta_margin = 40, rho_margin = 20, roi=finding_rect)
        if lines :
            for l in lines :
                img.draw_line(l.line() , color=(0,255,0))
        lineNum = len(lines)
        if lineNum == 3 :
            shapeType = 2
        elif lineNum == 4 :
            shapeType = 3
        elif lineNum == 2 :
            res = getShapeBy2Line(lines)
            shapeType = res[0]
            minlen = res[1]
            angle  = res[2]
            #print('line2')

        if len(lines)>2 and len(lines)<=8 :
            vtx = getVtxs(lines)
            for v in vtx:
                img.draw_circle(v[0],v[1],3,color=(255,255,0))
            if len(vtx)>2 :
                minlen = int(getMinVtxLen(vtx))

    img.draw_rectangle(finding_rect[0],finding_rect[1],finding_rect[2],finding_rect[3],color=(255,0,255))

    if laserPos!=None :
        img.draw_circle(laserPos[0],laserPos[1],2,color=(0,0,255))

    if blobs:
        for b in blobs:
            img.draw_rectangle(b[0:4])

    fontColor = (255,255,255)
    if haveLaser :
        fontColor = (0,0,255)

    if soccer_1 :
        for s in soccer_1 :
            img.draw_rectangle(s[0:4],color=(255,0,0))
    if soccer_2 :
        for s in soccer_1 :
            img.draw_rectangle(s[0:4],color=(0,0,255))
    if bask :
        for b in bask :
            img.draw_rectangle(b[0:4] , color=(0,255,0))

    target_size = 0
    if shapeType == 1 :
        cir_l = cir_r*2
        img.draw_string(0, 0, "cir l="+str(int(cir_l))+'pix', scale=1 , color = fontColor)
        target_size = int(getLenByHCPX(hc_len,cir_l))
        img.draw_string(0, 10, "real r="+str(target_size)+'cm', scale=1 , color = (128,255,128))
    elif shapeType == 2 :
        img.draw_string(0, 0, "tri l="+str(int(minlen))+'pix', scale=1 , color = fontColor)
        target_size = int(getLenByHCPX(hc_len,minlen))
        img.draw_string(0, 10, "real size="+str(target_size)+'cm', scale=1 , color = (128,255,128))
    elif shapeType == 3 :
        img.draw_string(0, 0, "rect l="+str(int(minlen))+'pix', scale=1 , color = fontColor)
        target_size = int(getLenByHCPX(hc_len,minlen))
        img.draw_string(0, 10, "real size="+str(target_size)+'cm', scale=1 , color = (128,255,128))

    if ballType == 1:
        img.draw_string(0, 180, "soccerball", scale=1 , color = (255,128,128))
    elif ballType == 2:
        img.draw_string(0, 180, "basketball", scale=1 , color = (255,128,128))
    elif ballType == 3:
        img.draw_string(0, 180, "volleyball", scale=1 , color = (255,128,128))

    if not lock_ignore_3D :
        img.draw_string(100, 180, "find 3d", scale=1 , color = (255,128,255))

    if cam_move == 1:
        img.draw_string(150, 210, 'up', scale=1 , color = (255,255,128))
    elif cam_move == 2:
        img.draw_string(150, 210, 'down', scale=1 , color = (255,255,128))
    elif cam_move == 3:
        img.draw_string(150, 210, 'left', scale=1 , color = (255,255,128))
    elif cam_move == 4:
        img.draw_string(150, 210, 'right', scale=1 , color = (255,255,128))

    if uart.read(100) :
        img.draw_string(150, 200, 'servo', scale=1 , color = (255,0,0))
        if cam_move == 1:
            uart.write('u')
        elif cam_move == 2:
            uart.write('d')
        elif cam_move == 3:
            uart.write('l')
        elif cam_move == 4:
            uart.write('r')
        else:
            uart.write('o')

    if fetched_color:
        fetched_lab = image.rgb_to_lab(fetched_color)
        img.draw_string(130, 190, "color="+str(fetched_lab[0])+' '+str(fetched_lab[1])+' '+str(fetched_lab[2]), scale=1 , color = (128,255,255)

    img.draw_string(0, 190, "box_size="+str(min(finding_size[0],finding_size[1]))+'pix', scale=1 , color = (128,255,128))
    img.draw_string(0, 210, "hc="+str(int(hc_len))+'cm', scale=1 , color = (128,255,128))
    img.draw_string(0, 220, "line="+str(lineNum), scale=1 , color = (128,255,128))
    if angle!=None :
        img.draw_string(100, 220, "angle="+str(angle), scale=1 , color = (128,255,128))

    lcd.display(img)

    if math.fabs(process_startTime - utime.ticks_ms())>10000 and cam_move == 0 and (lock_ignore_3D or ballType != 0):
        if shapeType != 0 and target_size>5 and hc_len<600:
            process_startTime = utime.ticks_ms()
            sound.value(1)
            utime.sleep_ms(100)
            sound.value(0)
            while button.value() == 1 :
                pass
            while button.value() == 0 :
                pass
            utime.sleep_ms(1000)
            process_startTime = utime.ticks_ms()
