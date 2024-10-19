/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;

import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.sensor.FtcPinpointOdometry;
import ftclib.sensor.FtcSparkFunOtos;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.vision.TrcHomographyMapper;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains Gobilda motor parameters.
     */
    public static class Gobilda
    {
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_84_ENC_PPR        =
            (((1.0 + 46.0/17.0) * (1.0 + 46.0/17.0) * (1.0 + 46.0/11.0)) * 28.0);
        public static final double MOTOR_5203_84_MAX_RPM        = 84.0;
        public static final double MOTOR_5203_84_MAX_VEL_PPS    =
            MOTOR_5203_84_ENC_PPR * MOTOR_5203_84_MAX_RPM / 60.0;     // 2789.661 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_312_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
        public static final double MOTOR_5203_312_MAX_RPM       = 312.0;
        public static final double MOTOR_5203_312_MAX_VEL_PPS   =
            MOTOR_5203_312_ENC_PPR * MOTOR_5203_312_MAX_RPM / 60.0;     // 2795.9872 pps
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double MOTOR_5203_435_ENC_PPR       = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
        public static final double MOTOR_5203_435_MAX_RPM       = 435.0;
        public static final double MOTOR_5203_435_MAX_VEL_PPS   =
            MOTOR_5203_435_ENC_PPR * MOTOR_5203_435_MAX_RPM / 60.0;     // 2787.9135 pps
    }   //class Gobilda

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        // DO NOT CHANGE the AprilTag location numbers. They are from the AprilTag metadata.
        // All AprilTags are at the height of 5.75-inch from the tile floor.
        public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
        public static final double APRILTAG_BACK_WALL_X             = 70.25;
        public static final double APRILTAG_BLUE_ALLIANCE_WALL_Y    = 70.25;
        public static final double APRILTAG_RED_ALLIANCE_WALL_Y     = -70.25;
        public static final double APRILTAG_WALL_OFFSET_Y           = 46.83;
        public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, APRILTAG_WALL_OFFSET_Y, -90.0), // TagId 11
            new TrcPose2D(0.0, APRILTAG_BLUE_ALLIANCE_WALL_Y, 0.0),                 // TagId 12
            new TrcPose2D(APRILTAG_BACK_WALL_X, APRILTAG_WALL_OFFSET_Y, 90.0),      // TagId 13
            new TrcPose2D(APRILTAG_BACK_WALL_X, -APRILTAG_WALL_OFFSET_Y, 90.0),     // TagId 14
            new TrcPose2D(0.0, APRILTAG_RED_ALLIANCE_WALL_Y, 180.0),                // TagId 15
            new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -APRILTAG_WALL_OFFSET_Y, -90.0) // TagId 16
        };

        public static final TrcPose2D RED_BASKET_SCORE_POSE         =
            new TrcPose2D(-2.0*Field.FULL_TILE_INCHES, -2.0*Field.FULL_TILE_INCHES, 225.0);
        public static final TrcPose2D BLUE_BASKET_SCORE_POSE        =
            new TrcPose2D(2.0*Field.FULL_TILE_INCHES, 2.0*Field.FULL_TILE_INCHES, 45.0);
        public static final double BASKET_LOW_EXTENDER_POS          = 40.0;
        public static final double BASKET_HIGH_EXTENDER_POS         = 80.0;
        public static final double BASKET_LOW_ELBOW_ANGLE           = 15.0;
        public static final double BASKET_HIGH_ELBOW_ANGLE          = 30.0;
        public static final double BASKET_LOW_WRIST_SCORE_POS       = 5.0;
        public static final double BASKET_HIGH_WRIST_SCORE_POS      = 10.0;

        public static final TrcPose2D RED_CHAMBER_SCORE_POSE        =
            new TrcPose2D(2.0*Field.FULL_TILE_INCHES, 0.0, 90.0);
        public static final TrcPose2D BLUE_CHAMBER_SCORE_POSE       =
            new TrcPose2D(-2.0*Field.FULL_TILE_INCHES, 0.0, 270.0);
        public static final double CHAMBER_LOW_EXTENDER_POS         = 40.0;
        public static final double CHAMBER_HIGH_EXTENDER_POS        = 80.0;
        public static final double CHAMBER_LOW_ELBOW_ANGLE          = 15.0;
        public static final double CHAMBER_HIGH_ELBOW_ANGLE         = 30.0;
        public static final double CHAMBER_LOW_WRIST_SCORE_POS      = 5.0; // intake not final
        public static final double CHAMBER_HIGH_WRIST_SCORE_POS     = 10.0; // intake not final

        // Robot start locations in inches.
        public static final double STARTPOS_X                       = 0.5 * Field.FULL_TILE_INCHES;
        public static final double STARTPOS_Y                       = Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0;
        public static final TrcPose2D STARTPOSE_RED_NET_ZONE        = new TrcPose2D(-STARTPOS_X, -STARTPOS_Y, 0.0);
        public static final TrcPose2D STARTPOSE_RED_OBSERVATION_ZONE= new TrcPose2D(STARTPOS_X, -STARTPOS_Y, 0.0);

    }   //class Game

    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             =
            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.1;      // in msec
        public static final String ROBOT_CODEBASE               = "Robot2025";
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
        public static final double ROBOT_LENGTH                 = 18.0;     // Measured in inches (CAD said 456 mm)
        public static final double ROBOT_WIDTH                  = 18.0;     // Measured in inches (CAD said 456 mm)
    }   //class Robot

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        IntoTheDeepRobot,
        CenterStageRobot,
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly
    }   //enum RobotType

    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.IntoTheDeepRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // User feedback
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showSubsystems              = true;
        public static final boolean useBlinkinLED               = false;
        public static final boolean useGobildaLED               = true;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean useWebCam                   = true;     // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useLimelightVision          = true;
        public static final boolean useCameraStreamProcessor    = false;
        public static final boolean useAprilTagVision           = false;
        public static final boolean useColorBlobVision          = true;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = true;
        // Drive Base
        public static final boolean useDriveBase                = true;
        public static final boolean usePinpointOdometry         = true;
        public static final boolean useSparkfunOTOS             = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useElbow                    = true;
        public static final boolean useExtender                 = true;
        public static final boolean useWrist                    = true;
        public static final boolean useAuxClimber               = false;
        public static final boolean useIntake                   = true;
        public static final boolean useGrabber                  = false;
    }   //class Preferences

    //
    // Robot Parameters.
    //

    /**
     * This class contains the parameters of the front camera.
     */
    public static class SampleCamParams extends FtcRobotDrive.VisionInfo
    {
        public SampleCamParams()
        {
            camName = "Webcam 1";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 2.0;                   // Inches forward from robot center
            camZOffset = 9.75;                  // Inches up from the floor
            camPitch = 15.0;                    // degrees down from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                             // Camera Top Left
                camImageWidth -1, 120.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - 9.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - 9.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - 9.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - 9.0 - camYOffset);    // World Bottom Right
        }   //SampleCamParams
    }   //class SampleCamParams

    /**
     * This class contains the parameters of the Limelight vision processor.
     */
    public static class LimelightParams extends FtcRobotDrive.VisionInfo
    {
        public LimelightParams()
        {
            camName = "Limelight3a";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0.0;                   // Inches forward from robot center
            camZOffset = 0.0;                   // Inches up from the floor
            camPitch = 0.0;                     // degrees down from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                             // Camera Top Left
                camImageWidth - 1, 120.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - 9.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - 9.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - 9.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - 9.0 - camYOffset);    // World Bottom Right
        }   //LimelightParams
    }   //class LimelightParams

    /**
     * This class contains the IntoTheDeep Robot Parameters.
     */
    public static class IntoTheDeepRobotParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER_MM = 35.0;
        private static final double ODWHEEL_DIAMETER = ODWHEEL_DIAMETER_MM * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public IntoTheDeepRobotParams()
        {
            robotName = "IntoTheDeepRobot";
            // Robot Dimensions (measured from CAD model if possible)
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = 360.0 * TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = (312.0 + 103.623) * TrcUtil.INCHES_PER_MM;
            // IMU (not used if using AbsoluteOdometry).
            imuName = null;
            hubLogoDirection = LogoFacingDirection.RIGHT;   // Control Hub orientation
            hubUsbDirection = UsbFacingDirection.UP;        // Control Hub orientation
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            // Odometry Wheels (Offset from wheel base center)
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};  // Used by OctoQuad only
            xOdWheelIndices = new int[] {0};    // Either motor port or OctoQuad port, not used by AbsoluteOdometry.
            xOdWheelXOffsets = new double[] {4.0 * TrcUtil.INCHES_PER_MM};
            xOdWheelYOffsets = new double[] {-132.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};   // Used by OctoQuad only
            yOdWheelIndices = new int[] {1, 2}; // Either motor port or OctoQuad port, not used by AbsoluteOdometry.
            yOdWheelXOffsets = new double[] {-156.0 * TrcUtil.INCHES_PER_MM, 156.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {-4.0 * TrcUtil.INCHES_PER_MM, -4.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                        .setPodOffsets(156.0, -144.0)   // Offsets from robot center
                        .setEncoderResolution(ODWHEEL_CPR / (Math.PI * ODWHEEL_DIAMETER_MM))
                        .setEncodersInverted(false, false);
                    absoluteOdometry = new FtcPinpointOdometry("pinpointOdo", ppOdoConfig);
                    headingWrapRangeLow = -180.0;
                    headingWrapRangeHigh = 180.0;
                }
                else if (RobotParams.Preferences.useSparkfunOTOS)
                {
                    FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                        .setOffset(-4.0 * TrcUtil.INCHES_PER_MM, 24.0 * TrcUtil.INCHES_PER_MM, 0.0)
                        .setScale(1.0, 1.0);    //???
                    absoluteOdometry = new FtcSparkFunOtos("sparkfunOtos", otosConfig);
                }
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry only (not used).
            xDrivePosScale = 1.0;   // in/count
            yDrivePosScale = 1.0;   // in/count
            // TODO: Tune everything below here.
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec   TODO: tune this
            robotMaxAcceleration  = 500.0;  // inches/sec2  TODO: tune this
            robotMaxTurnRate = 100.0;       // degrees/sec  TODO: tune this
            profiledMaxVelocity = robotMaxVelocity;         // TODO: tune this
            profiledMaxAcceleration = robotMaxAcceleration; // TODO: tune this
            profiledMaxTurnRate = robotMaxTurnRate;         // TODO: tune this
            // DriveBase PID Parameters
            drivePidTolerance = 0.5;
            turnPidTolerance = 0.5;
            xDrivePidCoeffs = new PidCoefficients(0.055, 0.0, 0.00001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.015, 0.0, 0.0005, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.028, 0.1, 0.0025, 0.0, 5.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;    // TODO: check this
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);  // TODO: check this
            // Vision
            webCam1 = new SampleCamParams();
            webCam2 = null;
            limelight = new LimelightParams();
            // Miscellaneous
            indicatorName = "gobildaLED";
        }   //IntoTheDeepRobotParams
    }   //class IntoTheDeepRobotParams

    /**
     * This class contains the CenterStage Robot Parameters.
     */
    public static class CenterStageRobotParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public CenterStageRobotParams()
        {
            robotName = "CenterStageRobot";
            // Robot Dimensions
            robotLength = 18.0;
            robotWidth = 17.25;
            wheelBaseLength = (24.0 * 15) * TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 390.0 * TrcUtil.INCHES_PER_MM;
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.LEFT;
            hubUsbDirection = UsbFacingDirection.UP;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;     // Used by OctoQuad only.
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};   // motor port used.
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;     // Used by OctoQuad only.
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};  //motor port
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, 144.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {-12.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry only (not used).
            xDrivePosScale = 1.0;   // in/count
            yDrivePosScale = 1.0;   // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 72.0;        // inches/sec
            robotMaxAcceleration  = 530.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 2.0;
            xDrivePidCoeffs = new PidCoefficients(0.095, 0.0, 0.006, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.035, 0.0, 0.0035, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.012, 0.0, 0.0012, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new SampleCamParams();
            webCam2 = null;
            limelight = null;
            // Miscellaneous
            indicatorName = "blinkin";
        }   //CenterStageRobotParams
    }   //class CenterStageRobotParams

    public static class VisionOnlyParams extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            webCam1 = new SampleCamParams();
            webCam2 = null;
            limelight = new LimelightParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    //
    // Subsystems.
    //

    public static class ElbowParams
    {
        public static final String SUBSYSTEM_NAME               = "Elbow";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_INVERTED        = true;

        public static final double ENCODER_CPR                  = Gobilda.MOTOR_5203_84_ENC_PPR;
        public static final double GEAR_RATIO                   = 44.0 / 10.0;
        public static final double DEG_SCALE                    = 360.0 / (ENCODER_CPR * GEAR_RATIO);
        public static final double POS_OFFSET                   = 0.0;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.2;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 120.0;
        public static final double GROUND_PICKUP_POS            = MIN_POS;
        public static final double[] posPresets                 = {MIN_POS, 30.0, 60.0, 90.0, MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.12, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.2 / ExtenderParams.POS_OFFSET;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class ElbowParams

    public static class ExtenderParams
    {
        public static final String SUBSYSTEM_NAME               = "Extender";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_INVERTED        = false;

        public static final double INCHES_PER_COUNT             = 0.0020825894713429496;
        public static final double POS_OFFSET                   = 16.25;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.75;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 35.0;
        public static final double GROUND_PICKUP_POS            = MIN_POS;
        public static final double[] posPresets                 = {MIN_POS, 20.0, 25.0, 30.0, 35.0};
        public static final double POS_PRESET_TOLERANCE         = 3.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(5.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.1;
        public static final double GRAVITY_COMP_POWER           = 0.0;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class ExtenderParams

    public static class WristParams
    {
        public static final String SUBSYSTEM_NAME               = "Wrist";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final double MIN_POS                      = 0.0;
        public static final double MAX_POS                      = 90.0;
        public static final double GROUND_PICKUP_POS            = MIN_POS;
        public static final double DUMP_TIME                    = 0.5;
    }   //class WristParams

    public static class ClimberParams
    {
        public static final String SUBSYSTEM_NAME               = "Climber";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_INVERTED        = false;

        public static final double INCHES_PER_COUNT             = 18.25/4941.0;
        public static final double POS_OFFSET                   = 10.875;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 30.25;
        public static final double[] posPresets                 = {MIN_POS, MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 1.0;
    }   //class ClimberParams

    public static final class IntakeParams
    {
        public static final String SUBSYSTEM_NAME               = "Intake";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.CRServo;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;
        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final MotorType FOLLOWER_MOTOR_TYPE       = MotorType.CRServo;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = !PRIMARY_MOTOR_INVERTED;

        public static final String SENSOR_NAME                  = SUBSYSTEM_NAME + ".sensor";
        public static final boolean SENSOR_INVERTED             = false;
        public static final double[] SENSOR_THRESHOLDS          = new double[] {1.3};

        public static final double FORWARD_POWER                = 1.0;
        public static final double REVERSE_POWER                = -0.5;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class IntakeParams

    public static final class GrabberParams
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final double OPEN_POS                     = 0.4;
        public static final double OPEN_TIME                    = 0.6;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;

        public static final boolean USE_ANALOG_SENSOR           = false;
        public static final String ANALOG_SENSOR_NAME           = SUBSYSTEM_NAME + ".sensor";
        public static final double SENSOR_TRIGGER_THRESHOLD     = 2.0;
        public static final double HAS_OBJECT_THRESHOLD         = 2.0;
        public static final boolean ANALOG_TRIGGER_INVERTED     = true;

        public static final boolean USE_DIGITAL_SENSOR          = false;
        public static final String DIGITAL_SENSOR_NAME          = SUBSYSTEM_NAME + ".sensor";
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;
    }   //class GrabberParams

}   //class RobotParams
