-- This is the Epuck principal control script. It is threaded

actualizeLEDs=function()
    if (relLedPositions==nil) then
        relLedPositions={{-0.0343,0,0.0394},{-0.0297,0.0171,0.0394},{0,0.0343,0.0394},
                    {0.0297,0.0171,0.0394},{0.0343,0,0.0394},{0.0243,-0.0243,0.0394},
                    {0.006,-0.0338,0.0394},{-0.006,-0.0338,0.0394},{-0.0243, -0.0243,0.0394}}
    end
    if (drawingObject) then
        sim.removeDrawingObject(drawingObject)
    end
    type=sim.drawing_painttag+sim.drawing_followparentvisibility+sim.drawing_spherepoints+
        sim.drawing_50percenttransparency+sim.drawing_itemcolors+sim.drawing_itemsizes+
        sim.drawing_backfaceculling+sim.drawing_emissioncolor
    drawingObject=sim.addDrawingObject(type,0,0,bodyElements,27)
    m=sim.getObjectMatrix(ePuckBase,-1)
    itemData={0,0,0,0,0,0,0}
    sim.setLightParameters(ledLight,0)
    for i=1,9,1 do
        if (ledColors[i][1]+ledColors[i][2]+ledColors[i][3]~=0) then
            p=sim.multiplyVector(m,relLedPositions[i])
            itemData[1]=p[1]
            itemData[2]=p[2]
            itemData[3]=p[3]
            itemData[4]=ledColors[i][1]
            itemData[5]=ledColors[i][2]
            itemData[6]=ledColors[i][3]
            sim.setLightParameters(ledLight,1,{ledColors[i][1],ledColors[i][2],ledColors[i][3]})
            for j=1,3,1 do
                itemData[7]=j*0.003
                sim.addDrawingObjectItem(drawingObject,itemData)
            end
        end
    end
end

-- Function to read the Light Sensors 
getLightSensors=function()
    data=sim.receiveData(0,'EPUCK_lightSens')
    if (data) then
        lightSens=sim.unpackFloatTable(data)
    end
    return lightSens
end

function sysCall_threadmain()
    -- Objects Instantiation
    sim.setThreadSwitchTiming(200) -- We will manually switch in the main loop
    -- ePuck body objects instantiation
    bodyElements=sim.getObjectHandle('ePuck_bodyElements')
    leftMotor=sim.getObjectHandle('ePuck_leftJoint')
    rightMotor=sim.getObjectHandle('ePuck_rightJoint')
    ePuck=sim.getObjectHandle('ePuck')
    ePuckBase=sim.getObjectHandle('ePuck_base')
    ledLight=sim.getObjectHandle('ePuck_ledLight')

    -- Scene objects instantiation
    -- Big Cylinder instantiation
    obstacle=sim.getObjectHandle('obstacle_1')

    -- ePuck's Camera instantiantion
    camera=sim.getObjectHandle('ePuck_camera')

    -- Scene walls instantiation
    wallBack=sim.getObjectHandle('wall_back')
    wallFront=sim.getObjectHandle('wall_front')
    wallLeft=sim.getObjectHandle('wall_left')
    wallRight=sim.getObjectHandle('wall_right')

    -- ePuck's Proximity Sensors instantiation
    proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,8,1 do
        proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
    end
    maxVel=120*math.pi/180
    ledColors={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}

    -- Braitenberg weights for the 4 front prox sensors (avoidance):
    braitFrontSens_leftMotor={1,2,-2,-1}
    -- Braitenberg weights for the 2 side prox sensors (following):
    braitSideSens_leftMotor={-1,0}
    -- Braitenberg weights for the 8 sensors (following):
    braitAllSensFollow_leftMotor={-3,-1.5,-0.5,0.8,1,0,0,-4}
    braitAllSensFollow_rightMotor={0,1,0.8,-0.5,-1.5,-3,-4,0}
    braitAllSensAvoid_leftMotor={0,0.5,1,-1,-0.5,-0.5,0,0}
    braitAllSensAvoid_rightMotor={-0.5,-0.5,-1,1,0.5,0,0,0}

    -- Here we execute the regular main code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        st=sim.getSimulationTime() -- function to get the simulation time
        velLeft=0
        velRight=0
        opMode=sim.getScriptSimulationParameter(sim.handle_self,'opMode')
        lightSens=getLightSensors()
        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.065*s -- minimum detection distance for the proximity sensors, based in the size ratio of the ePuck's body elements
        -- Initialization of the detection distance for the proximity sensors
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end
        -- Functions to check if the two front proxSens recognize the big Cylinder
        ps3cil=sim.checkProximitySensor(proxSens[3],obstacle)
        ps4cil=sim.checkProximitySensor(proxSens[4],obstacle)

        -- Functions to check if the two front, and two diagonal/front proxSens recognize the Front Wall
        ps2wf=sim.checkProximitySensor(proxSens[2],wallFront)
        ps3wf=sim.checkProximitySensor(proxSens[3],wallFront)
        ps4wf=sim.checkProximitySensor(proxSens[4],wallFront)
        ps5wf=sim.checkProximitySensor(proxSens[5],wallFront)

        -- Functions to check if the two front, and two diagonal/front proxSens recognize the Back Wall
        ps2wb=sim.checkProximitySensor(proxSens[2],wallBack)
        ps3wb=sim.checkProximitySensor(proxSens[3],wallBack)
        ps4wb=sim.checkProximitySensor(proxSens[4],wallBack)
        ps5wb=sim.checkProximitySensor(proxSens[5],wallBack)

        -- Functions to check if the two front, and two diagonal/front proxSens recognize the Left Wall
        ps2wl=sim.checkProximitySensor(proxSens[2],wallLeft)
        ps3wl=sim.checkProximitySensor(proxSens[3],wallLeft)
        ps4wl=sim.checkProximitySensor(proxSens[4],wallLeft)
        ps5wl=sim.checkProximitySensor(proxSens[5],wallLeft)

        -- Functions to check if the two front, and two diagonal/front proxSens recognize the Right Wall
        ps2wr=sim.checkProximitySensor(proxSens[2],wallRight)
        ps3wr=sim.checkProximitySensor(proxSens[3],wallRight)
        ps4wr=sim.checkProximitySensor(proxSens[4],wallRight)
        ps5wr=sim.checkProximitySensor(proxSens[5],wallRight)
        
        -- Functions to read the detection of the front and side proxSens
        sensorRead1=sim.readProximitySensor  (proxSens[1])
        sensorRead2=sim.readProximitySensor  (proxSens[2])
        sensorRead3=sim.readProximitySensor  (proxSens[3])
        sensorRead4=sim.readProximitySensor  (proxSens[4])
        sensorRead5=sim.readProximitySensor  (proxSens[5])
        sensorRead6=sim.readProximitySensor  (proxSens[6])
        
        -- Functions and logic for the ePuck to get back to the line after avoiding any obstacle in any position
        if lightSens and ((lightSens[1]<0.5) and (lightSens[2]<0.5) and (lightSens[3]<0.5)) then   
            sensDist6=sim.readProximitySensor(proxSens[6]) 
            sensDist1=sim.readProximitySensor(proxSens[1]) 
            if(sensDist1 > 0) then
                totalTime=sim.getSimulationTime()
                now = totalTime
                interval= 0.8
                while (now-totalTime<(interval)) do
                    velLeft = 1
                    velRight= -1 
                    sim.setJointTargetVelocity(leftMotor,velLeft)
                    sim.setJointTargetVelocity(rightMotor,velRight)
                    now=sim.getSimulationTime()
                    sim.switchThread()
                end 
            end
            if (sensDist6 > 0) then
               totalTime=sim.getSimulationTime()
               now = totalTime
               interval= 0.8
               while (now-totalTime<(interval)) do
                   velLeft = -1
                   velRight= 1 
                   sim.setJointTargetVelocity(leftMotor,velLeft)
                   sim.setJointTargetVelocity(rightMotor,velRight)
                   now=sim.getSimulationTime()
                   sim.switchThread()
               end 
            end
        end

        -- Functions and logic to recognize the line when walking astray
        if lightSens and ((lightSens[1]<0.5) and (lightSens[2]<0.5) and (lightSens[3]<0.5)) then
            if proxSens and (sensorRead1 == 0) and ((sensorRead2 == 0) and (sensorRead3 == 0) and (sensorRead4 == 0) and (sensorRead5 == 0) and (sensorRead6 == 0)) then
                totalTime=sim.getSimulationTime()
                now = totalTime
                interval= 2
                while (now-totalTime<(interval)) do
                    -- After recognizing the line, the ePuck must rotate to start following the line
                    velLeft = -1
                    velRight= 1 
                    sim.setJointTargetVelocity(leftMotor,velLeft)
                    sim.setJointTargetVelocity(rightMotor,velRight)
                    now=sim.getSimulationTime()
                    sim.switchThread()
                end        
            end
        end

        if (opMode==0) then -- We wanna follow the line
            if (math.mod(st,2)>1.5) then
                intensity=1
            else
                intensity=0
            end
            for i=1,9,1 do
                ledColors[i]={intensity,0,0} -- red
            end
            -- Now make sure the light sensors have been read, we have a line and the 4 front prox. sensors didn't detect anything:
            if lightSens and ((lightSens[1]<0.5)or(lightSens[2]<0.5)or(lightSens[3]<0.5)) and (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                if (lightSens[1]>0.5) then 
                    velLeft=maxVel
                else
                    velLeft=maxVel*0.25
                end
                if (lightSens[3]>0.5) then 
                    velRight=maxVel
                else
                    velRight=maxVel*0.25
                end
            else
                velRight=maxVel
                velLeft=maxVel
                if (proxSensDist[2]+proxSensDist[3]+proxSensDist[4]+proxSensDist[5]==noDetectionDistance*4) then
                    -- Nothing in front. Maybe we have an obstacle on the side, in which case we wanna keep a constant distance with it:
                    if (proxSensDist[1]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[1]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[1]/noDetectionDistance))
                    end
                    if (proxSensDist[6]>0.25*noDetectionDistance) then
                        velLeft=velLeft+maxVel*braitSideSens_leftMotor[2]*(1-(proxSensDist[6]/noDetectionDistance))
                        velRight=velRight+maxVel*braitSideSens_leftMotor[1]*(1-(proxSensDist[6]/noDetectionDistance))
                    end
                else
                    -- Obstacle in front
                    velLeft=0
                    velRight=0
                    -- Logic to check if the sensors recognize the Left Wall, and if they do, stop the ePuck ant turn it around
                    if (ps2wl==1 or ps3wl==1 or ps4wl==1 or ps5wl==1) then
                        velLeft=0
                        velRight=0
                        totalTime=sim.getSimulationTime()
                        now = totalTime
                        interval= 3.5
                        while (now-totalTime<(interval)) do
                            velLeft = -1
                            velRight= 1
                            sim.setJointTargetVelocity(leftMotor,velLeft)
                            sim.setJointTargetVelocity(rightMotor,velRight)
                            now=sim.getSimulationTime()
                            sim.switchThread()
                        end
                    else
                        -- Same logic for the Right Wall
                        if (ps2wr==1 or ps3wr==1 or ps4wr==1 or ps5wr==1) then
                            velLeft=0
                            velRight=0
                            totalTime=sim.getSimulationTime()
                            now = totalTime
                            interval= 3.5
                            while (now-totalTime<(interval)) do
                                velLeft = 1
                                velRight= -1
                                sim.setJointTargetVelocity(leftMotor,velLeft)
                                sim.setJointTargetVelocity(rightMotor,velRight)
                                now=sim.getSimulationTime()
                                sim.switchThread()
                            end
                        else
                            -- Same logic for the Front Wall
                            if (ps2wf==1 or ps3wf==1 or ps4wf==1 or ps5wf==1) then
                                velLeft=0
                                velRight=0
                                totalTime=sim.getSimulationTime()
                                now = totalTime
                                interval= 3.5
                                while (now-totalTime<(interval)) do
                                    velLeft = 1
                                    velRight= -1
                                    sim.setJointTargetVelocity(leftMotor,velLeft)
                                    sim.setJointTargetVelocity(rightMotor,velRight)
                                    now=sim.getSimulationTime()
                                    sim.switchThread()
                                end
                            else
                                -- Same logic for the Back wall
                                if (ps2wb==1 or ps3wb==1 or ps4wb==1 or ps5wb==1) then
                                    velLeft=0
                                    velRight=0
                                    totalTime=sim.getSimulationTime()
                                    now = totalTime
                                    interval= 3.5
                                    while (now-totalTime<(interval)) do
                                        velLeft = -1
                                        velRight= 1
                                        sim.setJointTargetVelocity(leftMotor,velLeft)
                                        sim.setJointTargetVelocity(rightMotor,velRight)
                                        now=sim.getSimulationTime()
                                        sim.switchThread()
                                    end
                                else
                                    -- Similar logic for the big cylinder, but when recognized, the ePuck must stop
                                    if (ps3cil==1 or ps4cil==1) then
                                        velLeft=0
                                        velRight=0
                                    else
                                        -- If none of the Walls, or the Big Cylinder, were recognized, it means any other obstacle is there.
                                        -- So it must use Braitenberg to avoid it.
                                        for i=1,4,1 do
                                            velLeft=velLeft+maxVel*braitFrontSens_leftMotor[i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                                            velRight=velRight+maxVel*braitFrontSens_leftMotor[5-i]*(1-(proxSensDist[1+i]/noDetectionDistance))
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        actualizeLEDs()
        sim.switchThread() -- Don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

function sysCall_cleanup()
    -- Put some clean-up code here:
    for i=1,9,1 do
        ledColors[i]={0,0,0} -- no light
    end
    actualizeLEDs()
end