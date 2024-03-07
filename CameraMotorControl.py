###Imports (add or remove as needed)

import PyCapture2 ##module needed for the camera, this is located in the PyCapture 2 folder along with some examples & documentation
import matplotlib.pyplot as plt
import serial
import re
import pandas
import sys
import numpy as np
from PIL import Image as im
import kohzu_control as control ##file written by Connor which contains functions that control the kohzu motors and read their positions
import time
from instrumental import errors



##CAN TRY USING TO TROUBLESHOOT KOHZU MOTORS, SEE TROUBLESHOOTING SECTION OF REPORT
#ser = serial.Serial('com3', baudrate=9600)
#ser.close()


def print_build_info():
    '''This is a pre-written function from the PyCapture example files. 
    It tells us the PyCapture library, I mainly called it at the very start to ensure that the camera was connected properly'''
    lib_ver = PyCapture2.getLibraryVersion()
    print('PyCapture2 library version: %d %d %d %d' % (lib_ver[0], lib_ver[1], lib_ver[2], lib_ver[3]))
    print()


def print_camera_info(cam):
    '''This is a pre-written function from the PyCapture example files. 
    It tells us some camera info, I mainly called it at the very start to ensure that the camera was connected was the right one'''  
    cam_info = cam.getCameraInfo() ##What and where is this function from
    print('\n*** CAMERA INFORMATION ***\n')
    print('Serial number - %d' % cam_info.serialNumber)
    print('Camera model - %s' % cam_info.modelName)
    print('Camera vendor - %s' % cam_info.vendorName)
    print('Sensor - %s' % cam_info.sensorInfo)
    print('Resolution - %s' % cam_info.sensorResolution)
    print('Firmware version - %s' % cam_info.firmwareVersion)
    print('Firmware build time - %s' % cam_info.firmwareBuildTime)
    print()

#This Works
def enable_embedded_timestamp(cam, enable_timestamp):
    '''This is a pre-written function from the PyCapture example files. 
    It sets up the a stopwatch that starts when we call this function. I didn't use this much.'''
    embedded_info = cam.getEmbeddedImageInfo()
    if embedded_info.available.timestamp:
        cam.setEmbeddedImageInfo(timestamp = enable_timestamp)
        if enable_timestamp :
            print('\nTimeStamp is enabled.\n')
        else:
            print('\nTimeStamp is disabled.\n')

def grab_images(cam, name): #takes & saves an image file 
    '''This is similar to the grab_images pre-written function from the PyCapture example files. 
    I modified it to only take and save one picture (like in a typical camera) instead of retrieving/taking a bunch and only save the last one'''
    prev_ts = None
    try:
        image = cam.retrieveBuffer() 
    except PyCapture2.Fc2error as fc2Err:
        print('Error retrieving buffer : %s' % fc2Err)
           
        ts = image.getTimeStamp()
        if prev_ts:
            diff = (ts.cycleSeconds - prev_ts.cycleSeconds) * 8000 + (ts.cycleCount - prev_ts.cycleCount)
            print('Timestamp [ %d %d ] - %d' % (ts.cycleSeconds, ts.cycleCount, diff))
        prev_ts = ts

    newimg = image.convert(PyCapture2.PIXEL_FORMAT.MONO8) ##keep this at MONO8 since our camera only takes greyscale images
    print('Saving this image')
    newimg.save(name.encode('utf-8'), PyCapture2.IMAGE_FILE_FORMAT.PGM) ##this saves it as a .PGM but to save it as another file type,
                                                                        ##change it using the description on pg. 6 in the PyCapture documentation


def auto_properties():
    '''This is a function I made to set all of the camera parameters (you can see them if you click the icon to the right of the save button in FlyCap) to auto 
    so it will automatically reset the parameters based on its internal algorithm. You can't change a parameter value when it is set to auto. Checkbox will be checked if its in auto'''
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.AUTO_EXPOSURE, autoManualMode = True)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.PAN, autoManualMode = True)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.TILT, autoManualMode = True)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.SHUTTER, autoManualMode = True)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.GAIN, autoManualMode = True)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode = True)

def manual_properties():
    '''This is a function I made to set all of the camera parameters (you can see them if you click the icon to the right of the save button in FlyCap) to manual. 
    You must change the parameter to manual in order to reset it. The checkbox next to the parameter will become unchecked if its not auto.'''
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.AUTO_EXPOSURE, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.PAN, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.TILT, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.SHUTTER, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.GAIN, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode = False)

def grab_image_return_array(): 
    '''This is a function I made that takes an image (does not modify any parameters) and returns the image as an array for use in the hdr() algorithm.'''
    prev_ts = None
    try:
        image = c.retrieveBuffer() ##What & where is the retrieveBuffer function
    except PyCapture2.Fc2error as fc2Err:
        print('Error retrieving buffer : %s' % fc2Err)
           
        ts = image.getTimeStamp()
        if prev_ts:
            diff = (ts.cycleSeconds - prev_ts.cycleSeconds) * 8000 + (ts.cycleCount - prev_ts.cycleCount)
            print('Timestamp [ %d %d ] - %d' % (ts.cycleSeconds, ts.cycleCount, diff))
        prev_ts = ts

    newimg = image.convert(PyCapture2.PIXEL_FORMAT.MONO8)
    print('Saving this image')
    newimg.save("image.pgm".encode('utf-8'), PyCapture2.IMAGE_FILE_FORMAT.PGM) 

    array = read_pgm("image.pgm")
    array = np.array(array, dtype = np.int16)
    return array


def grab_image_return_array_eg(shutter, gain):
    '''This function is very similar to the grab_images_return_array function I made above. Only it changes the shutter & gain before taking the picture. 
    It is used heavily in the hdr() function/algorithm below'''
    #set the shutter and gain to the new parameters we want 
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.SHUTTER, absValue = shutter, autoManualMode = False)
    time.sleep(3)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.GAIN, absValue = gain, autoManualMode = False)
    time.sleep(3)

    #take the actual image
    grab_images(c, "image.pgm")

    #convert the image into a numpy array
    array = read_pgm("image.pgm")
    array = np.array(array, dtype = np.int16)
    return array


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array. This function was written by Connor. 

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.flip(np.array(np.frombuffer(buffer,
                         dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                         count=int(width)*int(height)#, offset=len(header)
                         ).reshape((int(height), int(width)))
                            ), axis=0)


def hdr() -> np.ndarray: 
    '''This algorithm is from a previous co-op student that I modified for this experiment. It basically compensates for the fact that the camera can only register high light intensites to a certain limit (which our experiment exceeds). 
    See more details in the hdr() section in the report. '''

    #auto_properties() #reset everything to auto/default, use this if the default parameters give you a nice, clear image


    c.setProperty(type = PyCapture2.PROPERTY_TYPE.GAIN, absValue = 9.0, autoManualMode = False) #use this to set the parameters to more fitting values if the 
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.SHUTTER, absValue = 67.0, autoManualMode = False) #default parameters do not give a nice, clear image
    time.sleep(3) #allow camera to adjust

    
    image_matrix = grab_image_return_array() #take our initial image 
    shutter = c.getProperty(12).absValue #get the starting shutter & gain values for the algorithm portion next
    gain = c.getProperty(13).absValue
    print("shutter = " + str(shutter))
    print("gain = " + str(gain))


    # Overexposed pixel compensation
    exposure_scale = 1
    iteration_counter = 0
    overexposed_matrix_bool = image_matrix >= 255 ##can change this pixel value to anything between 0-255 (range of possible pixel values)depending on what you're trying to do
    
    while np.sum(overexposed_matrix_bool): ##while we still have overexposed pixels 
        '''The following few lines of commented code basically increase the rate at which we increase the exposure scale to speed up the algorithm. 
        This means they are optional/don't really affect the algorithm. I found that they weren't really needed for our setup'''
        #if iteration_counter > 10: #stop if we iterate too much
            #break
        #if np.sum(overexposed_matrix_bool) > 10000: 
           # exposure_scale *= 2
            #if np.sum(overexposed_matrix_bool) > 100000:
               # exposure_scale *= 2
        if shutter/exposure_scale < 1.0:  # Min exposure time, this can also be set to other values depending on what you're doing 
            print("HDR Error - Reached minimum exposure time")
            break

        exposure_scale *= 2  # halves exposure each iteration
        iteration_counter += 1
        image_matrix_underexposed = grab_image_return_array_eg(shutter/exposure_scale, gain) #changed this to my function
        print("requested shutter value = " + str(shutter/exposure_scale)) #tells us what our intended shutter value is for our next image
        print("actual shutter value = " + str(c.getProperty(12).absValue)) #tells us what our actual shutter value was (in case our camera has issues, see report)
        image_matrix[overexposed_matrix_bool] = (image_matrix_underexposed[overexposed_matrix_bool] * exposure_scale) #scale the images up by exposure value to compensate??? Strange
        overexposed_matrix_bool = image_matrix_underexposed >= 255 #update overexposed_matrix_bool
        print(
            f"HDR iteration {iteration_counter}: {np.sum(overexposed_matrix_bool)} pixels overexposed"
        )
    return image_matrix

def hdrlaser() -> np.ndarray:
    '''This is very similar to the hdr() function above. 
    The main difference is that it keeps the number of exposure scalings constant, which is needed if you are changing the laser power values each time. See report.'''
    
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.GAIN, absValue = 0.0, autoManualMode = False)
    c.setProperty(type = PyCapture2.PROPERTY_TYPE.SHUTTER, absValue = 18.0, autoManualMode = False) #lower the starting gain
    time.sleep(3)
    
    image_matrix = grab_image_return_array()
    shutter = c.getProperty(12).absValue #get the starting shutter & gain values
    gain = c.getProperty(13).absValue
    print("shutter = " + str(shutter))
    print("gain = " + str(gain))


    # Overexposed pixel compensation
    exposure_scale = 1
    iteration_counter = 0
    overexposed_matrix_bool = image_matrix >= 255
    
    while iteration_counter < 3: ##this keeps the number of exposure change iterations at 3
        exposure_scale *= 2  # halves exposure each iteration
        iteration_counter += 1
        image_matrix_underexposed = grab_image_return_array_eg(shutter/exposure_scale, gain) #changed this to my function
        print("requested shutter value = " + str(shutter/exposure_scale)) #tells us what our intended shutter value is for our next image
        print("actual shutter value = " + str(c.getProperty(12).absValue)) #tells us what our actual shutter value was (in case our camera has issues, see report)
        image_matrix[overexposed_matrix_bool] = (image_matrix_underexposed[overexposed_matrix_bool] * exposure_scale) #scale the images up by exposure value to compensate??? Strange
        overexposed_matrix_bool = image_matrix_underexposed >= 255 #update overexposed_matrix_bool
        print(
            f"HDR iteration {iteration_counter}: {np.sum(overexposed_matrix_bool)} pixels overexposed"
        )
    return image_matrix


def create_image_txt(image_matrix, image_name): 
    '''This is a function I made that takes in an image as a numpy array (the outputs of many of the above functions) and saves it as a regular photo file and text file.'''

    plt.imshow(image_matrix)
    plt.savefig(image_name+ ".png")
    np.set_printoptions(threshold=sys.maxsize)
    np.savetxt(image_name + ".txt", image_matrix)



    ############################### FUNCTION CALLS ##############################
    '''This is an example of commands one would call to take a set of phase stepping images moving just grating 1 along its x axis'''


    
print_build_info()

# # Ensure sufficient cameras are found
bus = PyCapture2.BusManager()
num_cams = bus.getNumOfCameras()
print('Number of cameras detected: ', num_cams)
if not num_cams:
    print('Insufficient number of cameras. Exiting...')
    exit()

# # Select camera and connect
c = PyCapture2.Camera()
uid = bus.getCameraFromIndex(0)
c.connect(uid)
print_camera_info(c)

# # Enable camera embedded timestamp
enable_embedded_timestamp(c, True)

'''Put an array of the image names (don't put the file type) you want in order below'''

image_names = ['0g1','6g1', '12g1', '18g1', '24g1', '30g1', '36g1', '42g1', 
               '48g1', '54g1', '60g1', '66g1', '72g1', '78g1', '84g1', 
               '90g1', '96g1', '102g1', '108g1', '114g1', '120g1', '126g1', 
               '132g1', '138g1', '144g1', '150g1', '156g1', '162g1', '168g1', 
               '174g1', '180g1', '186g1', '192g1', '198g1', '204g1', '210g1', 
               '216g1', '222g1', '228g1', '234g1', '240g1', '246g1', '252g1', 
               '258g1', '264g1', '270g1', '276g1', '282g1', '288g1', '294g1', 
               '300g1', '306g1', '312g1', '318g1', '324g1', '330g1', '336g1', 
               '342g1', '348g1', '354g1', '360g1'] #these values are in micrometers 
number_of_images = 61
image_ints = np.array(range(1, number_of_images + 1)) #this needs to be the same size as image_names


'''Set your starting positions in the axis you want to move in for all 3 gratings. These are in motor units'''

##FOR GRATINGS
g1xposition = 13200 + 720 #shift to make it go from -90 to 90 microns
g2xposition = -28000 + 720 #in motor units
g3xposition = 10000 + 720
interval = 24 ##in motor units

'''Move your gratings to the position specified above'''
control.kohzu.absolute_position_drive('g1_x_tra', 8, g1xposition) 
control.kohzu.absolute_position_drive('g2_x_tra', 8, g2xposition)
control.kohzu.absolute_position_drive('g3_x_tra', 8, g3xposition)


'''Confirm that the motors are in the right position'''
print("starting position for Grating 1: " + control.kohzu.absolute_position_read('g1_x_tra'))  
print("starting position for Grating 2: " + control.kohzu.absolute_position_read('g2_x_tra'))
print("starting position for Grating 3: " + control.kohzu.absolute_position_read('g3_x_tra'))

'''Prepare camera for image taking'''
print('Starting image capture...')
c.startCapture()
time.sleep(3)
manual_properties() #set properties to manual so that the values are held constant throughout

'''Run the image taking + motor movement process. This moves the motors and takes a picture for each image specificed in your image_names array.'''
for i in image_ints:
    image_matrix = hdr()
    create_image_txt(image_matrix, image_names[i-1])
    print(image_names[i-1]) # take the ith image
    print("Completed Image " + str(i) + "out of " + str(number_of_images))
    print("Position in Image " + str(i) + " is " + str(control.kohzu.absolute_position_read('g1_x_tra')))
    g1xposition = g1xposition - interval #increase by x motor units to prep for the next image
    g2xposition = g2xposition - interval
    g3xposition = g3xposition - interval
    control.kohzu.absolute_position_drive('g1_x_tra', 8, g1xposition) # move motors to prep for next image
    #control.kohzu.absolute_position_drive('g2_x_tra', 8, g2xposition)
    #control.kohzu.absolute_position_drive('g3_x_tra', 8, g3xposition)
    time.sleep(3) #give time to adjust


'''Tell camera it is done capturing images and disconnect'''
c.stopCapture()

# # Disable camera embedded timestamp
enable_embedded_timestamp(c, False)
c.disconnect()
input('Done! Press Enter to exit...\n')
