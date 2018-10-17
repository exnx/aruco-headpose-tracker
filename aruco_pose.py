import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import math
import socket

class Server:

    '''

    This class will setup a basic UDP server to stream the head pose numbers to a 
    receiving end.

    '''

    def __init__(self):
        self.UDP_IP = "127.0.0.1"
        self.UDP_PORT = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

    def send(self, data):

        '''

        This function will send the pose data to a socket.

        '''

        self.sock.sendto(data.encode(), (self.UDP_IP, self.UDP_PORT))

class Calibrate:

    '''

    This class extracts camera calibration information.  To use this, you will need to take 
    ~ 10 images with the camera you'll be using.  This will be important for getting accurate
     pose estimation.  You can use the default images provided, but it won't be super accurate
     in that case, but it will work.

    '''

    def calibrate(self, directory):

        '''

        This function uses the checkerboard images provided to extract the camera calibration info
        from the camera used.  

        '''

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((9*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob('{}/*.jpg'.format(directory))

        # reads image in dir
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (9,7),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (9,7), corners2, ret)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        return (mtx, dist, rvecs, tvecs)

class Pose:

    def __init__(self, camera_num):
        self.cap = cv2.VideoCapture(camera_num)
        self.server = Server()

    def stream_pose(self, calibrations, display=False):

        '''

        This function will use the calibration info, detect the aruco markers,
        and estimate the pose.  It will then send the pose to the UDP socket.

        '''

        mtx, dist, rvecs, tvecs = calibrations  # unpack tuple

        while (True):
            ret, frame = self.cap.read()
            # operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()

            #lists of ids and the corners beloning to each id
            markers, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

            if np.all(ids != None):

                curr_rx = 0
                curr_ry = 0
                curr_rz = 0

                for index, marker in enumerate(markers):

                    rvec, tvec,_ = aruco.estimatePoseSingleMarkers(markers[index], 0.04, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
                    #(rvec-tvec).any() # get rid of that nasty numpy value array error


                    # possible way to convert
                    # rx = np.degrees(rvec[0][0][0])
                    # ry = np.degrees(rvec[0][0][1])
                    # rz = np.degrees(rvec[0][0][2])

                    # new_rx = self.smooth(rx, curr_rx)
                    # new_ry = self.smooth(ry, curr_ry)
                    # new_rz = self.smooth(rz, curr_rz)

                    # curr_rx = int(new_rx)
                    # curr_ry = int(new_ry)
                    # curr_rz = int(new_rz)

                    # out = str(curr_rx) + ' ' + str(curr_ry) + ' ' + str(curr_rz)

                    rotation_matrix_zeros = np.zeros((3,3))
                    rotation_matrix = cv2.Rodrigues(rvec, rotation_matrix_zeros)
                    euler_angles = self.rotationMatrixToEulerAngles(rotation_matrix[0])

                    xrot, yrot, zrot = np.degrees(euler_angles)
                    xrot = int(xrot)  # green
                    yrot = int(yrot)  # blue
                    zrot = int(zrot)  # red

                    socket_out = 'marker# ' + str(index) + ' ' + str(xrot) + ' ' + str(yrot) + ' ' + str(zrot)
                    display_out = 'marker#: ' + str(index) + ' ' + 'pitch: ' + str(xrot) + ' yaw: ' + str(yrot) + ' roll: ' + str(zrot)

                    # send to server
                    self.server.send(socket_out)

                    cv2.putText(frame, display_out, (20,100), font, 1, (255, 0, 0), 2, cv2.LINE_AA)

                    aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
                    aruco.drawDetectedMarkers(frame, markers) #Draw A square around the markers

                    ###### DRAW ID #####
                    # cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

            if display:
                    # Display the resulting frame
                cv2.imshow('frame',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

    # -------------- helper functions ----------------#
    # used for converting the rotation matrix to euler angles

            # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
     
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self, R):

        assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    def smooth(self, new_reading, curr_state):
        c = .8
        new_est = (1-c) * curr_state + new_reading * c
        return new_est

def main(directory, camera_num, display):

    calibrator = Calibrate()
    calibrations = calibrator.calibrate(directory)
    
    poser = Pose(camera_num)
    poser.stream_pose(calibrations, display)

if __name__ == "__main__":

    # change parameters here

    camera_num = 1     # use 0 for default laptop webcam, or 1 for an external
    directory = 'checker-board-pics'  # provide directory for the calibration images
    display = True  # if you want to see the images with the pose

    main(directory, camera_num, display)


