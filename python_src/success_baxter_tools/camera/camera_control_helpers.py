#!/usr/bin/env python
import rospy
import rosgraph
import socket
import time
import errno
import os

from baxter_core_msgs.srv import OpenCamera
from baxter_core_msgs.srv import CloseCamera
from baxter_core_msgs.srv import ListCameras
from baxter_core_msgs.msg import CameraControl
from baxter_core_msgs.msg import CameraSettings

class CameraController(object):
    _validCameras = ['head_camera', 'left_hand_camera', 'right_hand_camera']
    _validRes = [(1280, 800),
                 (960, 600),
                 (640, 400),
                 (480, 300),
                 (384, 240),
                 (320, 200)]
    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    _autoControl = -1
    _defaultWidth = 320
    _defaultHeight = 200
    _defaultFps = 25

    @staticmethod
    def _getOpenCameras():
        poweredCameras = dict([(cam, False) for cam in CameraController._validCameras])
        streamingCameras = dict([(cam, False) for cam in CameraController._validCameras])

        # Using Cameras List service, get cameras that are recieving power
        rospy.wait_for_service('cameras/list')
        listService = rospy.ServiceProxy('cameras/list', ListCameras)
        try:
            resp = listService()
            for cam in resp.cameras:
                poweredCameras[cam] = True
        except rospy.ServiceException as err:
            raise OSError("error listing cameras: {}".format(err))

        # NOTE (amal): it turns out to be unnecessary to know the streamingCameras
        # given how Baxter RSDK 1.1 works.  Read the Readme for more details.
        # Using Rostopics, get the cameras that are currently streaming
        try:
            topics = rosgraph.Master('/rostopic').getPublishedTopics('/cameras')
            for topic in topics:
                for cam in CameraController._validCameras:
                    camTopic = "/cameras/%s/image" % cam
                    if camTopic == topic[0]:
                        streamingCameras[cam] = True
        except socket.error:
            raise ROSTopicIOException("Cannot communicate with master")

        return poweredCameras, streamingCameras

    @staticmethod
    def _isValidCameraSettings(settings):
        (w, h) = (settings.width, settings.height)
        if (w, h) not in CameraController._validRes:
            return False, "Invalid resolution (%d, %d), valid resolutions are %s" % (w, h, str(CameraController._validRes))
        isHalfResolution = False
        for control in settings.controls:
            if (control.id == CameraControl.CAMERA_CONTROL_EXPOSURE and
               (control.value < -1 or control.value > 100)):
               return False, "Invalid exposure %d" % control.value
            if (control.id == CameraControl.CAMERA_CONTROL_GAIN and
               (control.value < -1 or control.value > 79)):
               return False, "Invalid gain %d" % control.value
            if (control.id == CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R and
               (control.value < -1 or control.value > 4095)):
               return False, "Invalid white balance red %d" % control.value
            if (control.id == CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G and
               (control.value < -1 or control.value > 4095)):
               return False, "Invalid white balance green %d" % control.value
            if (control.id == CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B and
               (control.value < -1 or control.value > 4095)):
               return False, "Invalid white balance blue %d" % control.value
            if (control.id == CameraControl.CAMERA_CONTROL_RESOLUTION_HALF):
                isHalfResolution = control.value
        # Now that we know if it is half resolution, we can check the window
        limitX, limitY = 1280-settings.width, 800-settings.height
        if isHalfResolution:
            limitX /= 2
            limitY /= 2
        for control in settings.controls:
            if (control.id == CameraControl.CAMERA_CONTROL_WINDOW_X and
               (control.value < 0 or control.value > limitX)):
               return False, "Invalid window x %d, max = %d" % (control.value, limitX)
            if (control.id == CameraControl.CAMERA_CONTROL_WINDOW_Y and
               (control.value < 0 or control.value > limitY)):
               return False, "Invalid window y %d, max = %d" % (control.value, limitY)
        return True, ""


    @staticmethod
    def _setCameraController(controls, name, value):
        control = CameraControl()
        control.id = name
        control.value = value
        controls.append(control)

    @staticmethod
    def _getDefaultSettings():
        defaultSettings = CameraSettings()
        # Default camera setting on startup: http://sdk.rethinkrobotics.com/wiki/Cameras
        # defaultSettings.width = CameraController._defaultWidth
        # defaultSettings.height = CameraController._defaultHeight
        # defaultSettings.fps = CameraController._defaultFps
        # NOTE (amal): the below default values were removed because Baxter has
        # built-in defaults it sets internally, so I decided to defer to those.
        # # Auto Camera CameraController
        # controls = []
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_EXPOSURE, CameraController._autoControl)
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_GAIN, CameraController._autoControl)
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, CameraController._autoControl)
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G, CameraController._autoControl)
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B, CameraController._autoControl)
        # CameraController._setCameraController(controls, CameraControl.CAMERA_CONTROL_RESOLUTION_HALF, 1)
        # defaultSettings.controls = controls
        return defaultSettings

    @staticmethod
    def _addDefaultValues(camera, settings):
        # NOTE (amal): the below default values were removed because Baxter has
        # built-in defaults it sets internally, so I decided to defer to those.
        # if settings.width == 0:
        #     settings.width = CameraController._defaultWidth
        # if settings.height == 0:
        #     settings.height = CameraController._defaultHeight
        # if settings.fps == 0:
        #     settings.fps = CameraController._defaultFps
        if camera == 'head_camera':
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_FLIP, True)
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_MIRROR, True)

    @staticmethod
    def createCameraSettings(**kwargs):
        settings = CameraSettings()
        settings.controls = []
        if "width" in kwargs:
            settings.width = kwargs["width"]
        if "height" in kwargs:
            settings.height = kwargs["height"]
        if "fps" in kwargs:
            settings.fps = kwargs["fps"]
        if "exposure" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_EXPOSURE, kwargs["exposure"])
        if "gain" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_GAIN, kwargs["gain"])
        if "whiteBalanceR" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, kwargs["whiteBalanceR"])
        if "whiteBalanceG" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G, kwargs["whiteBalanceG"])
        if "whiteBalanceB" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B, kwargs["whiteBalanceB"])
        if "windowX" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_WINDOW_X, kwargs["windowX"])
        if "windowY" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_WINDOW_Y, kwargs["windowY"])
        if "flip" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_FLIP, kwargs["flip"])
        if "mirror" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_MIRROR, kwargs["mirror"])
        if "resolutionHalf" in kwargs:
            CameraController._setCameraController(settings.controls, CameraControl.CAMERA_CONTROL_RESOLUTION_HALF, kwargs["resolutionHalf"])
        return settings



    @staticmethod
    def openCameras(camera, camera2=None, settings=None, settings2=None):
        """
        Opens the specified camera, with settings, and camera2/settings2 if
        specified.  Cameras should be strings, settings should be instances of
        CameraSettings
        """
        # Check that cameras are valid
        if camera not in CameraController._validCameras:
            raise ValueError('invalid camera {}, valid cameras are{}s'.format(camera, str(CameraController._validCameras)))
        if camera2 is not None and camera2 not in CameraController._validCameras:
            raise ValueError('invalid camera2 {}, valid cameras are {}'.format(camera2, str(CameraController._validCameras)))

        # Check that the settings are valid
        if settings is not None:
            ok, err =  CameraController._isValidCameraSettings(settings)
            if not ok: raise ValueError("invalid settings: {}".format(err))
        else:
            settings = CameraController._getDefaultSettings()
        CameraController._addDefaultValues(camera, settings)
        if settings2 is not None:
            ok, err =  CameraController._isValidCameraSettings(settings2)
            if not ok: raise ValueError("invalid settings2: {}".format(err))
        else:
            settings2 = CameraController._getDefaultSettings()
        CameraController._addDefaultValues(camera2, settings2)

        # Get the currently powered cameras and what we need to close
        poweredCameras, streamingCameras = CameraController._getOpenCameras()
        numPoweredCameras = poweredCameras.values().count(True)
        numDesiredCameraStreaming = 0 # between camera and camera2, how many are open
        if streamingCameras[camera]:
            numDesiredCameraStreaming += 1
        if camera2 is not None and streamingCameras[camera2]:
            numDesiredCameraStreaming += 1

        # Close the necessary camera
        cameraToClose = None
        if camera2 is None:
            if not streamingCameras[camera] and numPoweredCameras >= 2:
                for cam, on in poweredCameras.iteritems():
                    if cam != camera:
                        cameraToClose = cam
                        break
        else:
            if numDesiredCameraStreaming < numPoweredCameras:
                for cam, on in poweredCameras.iteritems():
                    if cam != camera and cam != camera2:
                        cameraToClose = cam
                        break
        if cameraToClose is not None:
            CameraController.closeCamera(cameraToClose)

        # Open the specified cameras
        rospy.wait_for_service('cameras/open')
        openService = rospy.ServiceProxy('cameras/open', OpenCamera)
        try:
            resp = openService(camera, settings)
            if resp.err != 0 and resp.err != errno.EINVAL:
                raise OSError("error opening {}: {}".format(camera, os.strerror(resp.err)))
        except rospy.ServiceException as err:
            raise OSError("error opening {}: {}".format(camera, err))
        if camera2 is not None and camera2 != camera:
            try:
                resp = openService(camera2, settings2)
                if resp.err != 0 and resp.err != errno.EINVAL:
                    raise OSError("error opening {}: {}".format(camera2, os.strerror(resp.err)))
            except rospy.ServiceException as err:
                raise OSError("error opening {}: {}".format(camera2, os.strerror(resp.err)))


    @staticmethod
    def closeCamera(camera):
        rospy.wait_for_service('cameras/close')
        closeService = rospy.ServiceProxy('cameras/close', CloseCamera)
        try:
            resp = closeService(camera)
            if resp.err != 0 and resp.err != errno.EINVAL:
                raise OSError("error closing {}: {}".format(camera, os.strerror(resp.err)))
        except rospy.ServiceException as err:
            raise OSError("error closing {}: {}".format(camera, err))

# Sample Usage
if __name__ == '__main__':
    settings = CameraController.createCameraSettings(width=640, height=400, exposure=-1) # settings for the first camera
    settings2 = CameraController.createCameraSettings(width=1280, height=800, exposure=-1) # settings for the second camera
    CameraController.openCameras("head_camera", "right_hand_camera", settings=settings, settings2=settings2)
