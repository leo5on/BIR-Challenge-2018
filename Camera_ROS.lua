function sysCall_init()
        
    -- Instantiate the Objects:
    -- Instantiate the ePuck's camera
    camera=sim.getObjectHandle('ePuck_camera')
    -- Instantiate the Scene Passive Vision Sensor
    passiveVisionSensor=sim.getObjectHandle('PassiveVision_sensor')
    -- Instantiate the ROS Publisher and Subscriber
    pub=simROS.advertise('/image', 'sensor_msgs/Image')
    simROS.publisherTreatUInt8ArrayAsString(pub)
    sub=simROS.subscribe('/OpenCV_image', 'sensor_msgs/Image', 'imageMessage_callback')
    simROS.subscriberTreatUInt8ArrayAsString(sub)
end

function imageMessage_callback(msg)
    -- Apply the received image to the passive vision sensor that acts as an image container
    sim.setVisionSensorCharImage(passiveVisionSensor,msg.data)
end

function sysCall_sensing()
    -- Capture the image from the camera
    local data, w, h=sim.getVisionSensorCharImage(camera)
    d={}
    d['header']={seq=0,stamp=simROS.getTime(), frame_id='a'}
    d['height']=h
    d['width']=w
    d['encoding']='rgb8'
    d['is_bigendian']=1
    d['step']=w*3
    d['data']=data
    simROS.publish(pub, d)
end

function sysCall_cleanup()
    -- Shut down publisher and subscriber.
    simROS.shutdownPublisher(pub)
    simROS.shutdownSubscriber(sub)
end