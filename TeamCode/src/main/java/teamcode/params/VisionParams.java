/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.params;

import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.drivebase.FtcRobotDrive;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose3D;
import trclib.vision.TrcHomographyMapper;

public class VisionParams
{
    /**
     * This class contains the parameters of the front camera.
     */
    public static class SampleCam extends FtcRobotDrive.VisionInfo
    {
        public SampleCam()
        {
            camName = "Webcam 1";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = -4.25;                 // Inches to the right from robot center
            camYOffset = 5.5;                   // Inches forward from robot center
            camZOffset = 10.608;                // Inches up from the floor
            camYaw = 0.0;                       // degrees clockwise from robot front ???
            camPitch = 15.0;                    // degrees down from horizontal ???
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                19.0, 20.5,                     // Camera Top Left
                609.0, 43.0,                    // Camera Top Right
                85.0, 433.0,                    // Camera Bottom Left
                583.0, 445.0);                  // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -21.0, 45.25,                   // World Top Left
                20.5, 45.25,                    // World Top Right
                -4.25, 9.0,                     // World Bottom Left
                5.75, 9.0);                     // World Bottom Right
        }   //SampleCam
    }   //class SampleCam

    /**
     * This class contains the parameters of the Limelight vision processor.
     */
    public static class Limelight extends FtcRobotDrive.VisionInfo
    {
        public Limelight()
        {
            camName = "Limelight3a";
            camImageWidth = 640;
            camImageHeight = 480;
            camHFov = 54.5;                             // in degrees
            camVFov = 42.0;                             // in degrees
            camXOffset = 135.47*TrcUtil.INCHES_PER_MM;  // Inches to the right from robot center
            camYOffset = 2.073;                         // Inches forward from robot center
            camZOffset = 10.758;                        // Inches up from the floor
            camYaw = 0.0;                               // degrees clockwise from robot front
            camPitch = 0.0;                             // degrees down from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //Limelight
    }   //class Limelight

}   //class VisionParams
