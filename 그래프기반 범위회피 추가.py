import rospy
import math
import message_filters
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, Float64MultiArray 

x, y, psi = 0,0,0
lidardata = [99] * 360
psi_d=0
goalPos=[900,200]
Pos=[0,0]
ax = plt.subplot(121, polar = True)
ax1=plt.subplot(122,xlim=(-60,60),ylim=(0,1000))
PastPsi=0
data_range=np.linspace(0,2*np.pi, 360)
RighI=100
LeftI=100
PastPsi=0
boatSize=8
JPSIndex=[]
JPSList=[]
PhaseList=[]
D_gain=0
line=0.000000000005
discoverAngle=0
time=0
preFinal=0

def update(data1, data2, data3):
    global x, y, psi, lidardata,Pos
    x, y = data1.data[0], data1.data[1]
    psi = data2.data
    lidardata = data3.data
    Pos=[x,y]

def animate(i):
    global PastPsi
    global JPSIndex
    global JPSList
    ld = list(lidardata)
    #print(ld)
    for i in range(61, 300, 1):
        ld[i] = 0


            




















    
#############################################################################################################################################################목적지  설정
    Goal_Distance=np.sqrt(np.power(goalPos[0]-Pos[0],2)+np.power(goalPos[1]-Pos[1],2))
    Goal_Psi=np.arctan2(goalPos[0]-Pos[0],goalPos[1]-Pos[1])*180/np.pi-psi
    psi_d=int(Goal_Psi)
    if Goal_Psi<-180:
        Goal_Psi+=360
    elif Goal_Psi>180:
        Goal_Psi-=360
##############################################################################################################################################################목적지 설정 완료

#######################################################################################################################################################################
    lidarinv = [0] * 360
    FinalGoal=[0]*360

    for i in range(61, 300, 1):
        ld[i] = 0
    for i in range(360):
        if ld[i]>200:
            lidarinv[i]=1000
##########################################################################################################################################################################

    DistanceCost=np.zeros([120,1])
    AngleCost=np.zeros([120,1])
    Cost=np.zeros([120,1])
    for i in range(0,120):
        DistanceCost[i]=50000000*abs(1/(ld[i-60]+1)*(1/(ld[i-60]+1)))
        AngleCost[i]=0.1*abs(i - 60 - Goal_Psi)
    if psi<180:
        PastPsi=psi
    else:
        PastPsi=psi-360
    DistanceCostSketch=DistanceCost

 ##########################################################################################################################################################################################################################################3   

    Nowweight=0.8
    NearWeight=3
    for i in range(0,120):
        if i>0 and i<119:
            DistanceCost[i]=(NearWeight*DistanceCostSketch[i-1]+Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i+1])
            DistanceCostSketch[i]=DistanceCost[i]/(2*NearWeight+Nowweight)
        elif i==0:
            DistanceCost[i]=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i+1])
            DistanceCostSketch[i]=DistanceCost[i]/(Nowweight+NearWeight)
        else:
            DistanceCost[i]=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i-1])
            DistanceCostSketch[i]=DistanceCost[i]/(Nowweight+NearWeight)
    for i in range(119,-1,-1):
        if i>0 and i<119:
            DistanceCost[i]+=(NearWeight*DistanceCostSketch[i-1]+Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i+1])
            DistanceCostSketch[i]=DistanceCost[i]/(2*NearWeight+Nowweight)
            DistanceCost[i]/=2*(2*NearWeight+Nowweight)
        elif i==0:
            DistanceCost[i]+=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i+1])
            DistanceCostSketch[i]=DistanceCost[i]/(Nowweight+NearWeight)
            DistanceCost[i]/=2*(Nowweight+NearWeight)

        else:
            DistanceCost[i]+=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i-1])
            DistanceCostSketch[i]=(Nowweight*DistanceCostSketch[i]+NearWeight*DistanceCostSketch[i-1])/(Nowweight+NearWeight)
            DistanceCost[i]/=2*(Nowweight+NearWeight)
    Cost=DistanceCost+AngleCost
    #print(DistanceCost)

############################################################################################################################################################################################################################

    Smaller=[500000]
    FinalPsi=0
    final=0
    for i in range(0,120):
        if Cost[i]<Smaller[0]:
            Smaller[0]=Cost[i]
            final=i-60
            FinalPsi=i-60+psi
    
    if (lidardata[int(psi)] > Goal_Distance):
        FinalPsi=Goal_Psi+psi
        #print(lidardata[int(psi)], Goal_Distance)
    ControlFinal=int(FinalPsi-psi+60)


    W=psi-PastPsi

    if [x,y]==goalPos:
        FinalPsi=0


    global time
    global preFinal
    boatSize=10
    LeftI=0
    RighI=0
    LeftNum=0
    RighNum=0
    for i in range(-boatSize,boatSize+1):
        if lidardata[final+i]<100:
            if i<0:
                LeftI=abs(i)
                LeftNum+=1
            elif i>0:
                RighI=i
                RighNum+=1
            else:
                pass
    if LeftI==0 and RighI==0:
        pass
    elif LeftNum>RighNum:
        if LeftI>RighI:
            FinalPsi+=20
            if preFinal==0:
                preFinal=1
                preFinalAngle=FinalPsi
            print('s')
            if lidardata[i]<70:
                time=1
    elif RighNum>LeftNum:
        if RighI>LeftI:
            FinalPsi-=20
            if preFinal==0:
                preFinal=2
                preFinalAngle=FinalPsi
            print('s')
            if lidardata[i]<70:
                time=1
    if 0<time<2:
        if preFinal==1:
            FinalPsi=preFinalAngle
        if preFinal==0:
            FinalPsi=preFinalAngle
        time+=1
        print(FinalPsi)
    else:
        time=0
        preFinal=0

        


















        



    FinalGoal[int(FinalPsi-psi)]=500
##############################################################################################################################################################ros publish
    msg = Float32()
    msg.data = FinalPsi
    pub.publish(msg)
####################################################################################################################################################################################












    plotList=[]
    for i in range(-60,60):
        plotList.append(i)

##########################################################################################################################################################################################################################
    ax.clear()
    ax1.clear()
    ax.set_theta_zero_location('N')
    ax.plot([0,Goal_Psi*np.pi/180],[0,Goal_Distance],color="green",markersize=5)
    ax.plot(Goal_Psi*np.pi/180,Goal_Distance,'s',color="green",markersize=10)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.plot(data_range,ld,'o',color='r',markersize=1)
    #ax.plot(data_range,lidardata,'o',color='b',markersize=1)
    ax.fill(data_range,ld,"0.5")
    ax.fill(data_range,lidarinv,"0.9")
    ax.plot(0,0,'s',color='blue',markersize=15)
    ax.set_thetamin(-60)
    ax.set_thetamax(60)
    ax.set_rmax(500)
    ax.set_rmin(0)
    ax1.clear()
    ax1.axis([-60 , 60 , 0 , 1000])
    # ax1.plot(range(-60,61),np.concatenate([ld[300:], ld[:61]]), 'go', markersize = 2)
    ax1.plot(plotList,DistanceCost)
    ax1.plot(plotList,Cost,'red')
    ax1.plot(plotList,AngleCost,'green')
    ax.plot(data_range,FinalGoal,"1",markersize=30)
    ax1.plot(JPSIndex,JPSList,'bo', markersize = 10)
    JPSIndex=[]
    JPSList=[]
    

if __name__=="__main__":
    rospy.init_node("YPathPlan")
    pub=rospy.Publisher("test", Float32, queue_size=10)
    sub1=message_filters.Subscriber("GPSData" , Float64MultiArray)
    sub2=message_filters.Subscriber("IMUData" , Float32)
    sub3=message_filters.Subscriber("LidarData" , Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], 10, 0.5, allow_headerless=True)
    mf.registerCallback(update)
    ani=FuncAnimation(plt.gcf(),animate, interval=300)
    plt.show()
    ################################################################################################################################################################################################################################