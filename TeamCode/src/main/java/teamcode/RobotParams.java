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
import ftclib.drivebase.FtcSwerveDrive;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.sensor.FtcSparkFunOtos;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
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
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
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
        public static final double ROBOT_LENGTH                 = 17.0;
        public static final double ROBOT_WIDTH                  = 17.0;
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
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
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
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
        public static final RobotType robotType                 = RobotType.CenterStageRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useLoopPerformanceMonitor   = true;
        public static final boolean useBatteryMonitor           = false;
        // Status Update: Status Update may affect robot loop time, don't do it when in competition.
        public static final boolean doStatusUpdate              = !inCompetition;
        public static final boolean showSubsystems              = true;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean useWebCam                   = true;     // false to use Android phone camera.
        public static final boolean useBuiltinCamBack           = false;    // For Android Phone as Robot Controller.
        public static final boolean tuneColorBlobVision         = false;
        public static final boolean useAprilTagVision           = true;
        public static final boolean useColorBlobVision          = true;
        public static final boolean useLimelightVision          = false;
        public static final boolean showVisionView              = !inCompetition;
        public static final boolean showVisionStat              = true;
        // Drive Base
        public static final boolean useDriveBase                = false;
        // Subsystems
        public static final boolean useSubsystems               = false;
    }   //class Preferences

    //
    // Robot Parameters.
    //

    /**
     * This class contains the parameters of the front camera.
     */
    public static class FrontCamParams extends FtcRobotDrive.VisionInfo
    {
        public FrontCamParams()
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
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //FrontCamParams
    }   //class FrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class BackCamParams extends FtcRobotDrive.VisionInfo
    {
        public BackCamParams()
        {
            camName = "Webcam 2";
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
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //BackCamParams
    }   //class BackCamParams

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
                -12.5626, 48.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //LimelightParams
    }   //class LimelightParams

    /**
     * This class contains the IntoTheDeep Robot Parameters.
     */
    public static class IntoTheDeepRobotParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public IntoTheDeepRobotParams()
        {
            robotName = "IntoTheDeepRobot";
            // Robot Dimensions
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = null;//TrcDriveBase.OdometryType.AbsoluteOdometry;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};
            xOdWheelIndices = new int[] {0};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};
            yOdWheelIndices = new int[] {1, 2};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                    .setOffset(0.0, 0.0, 0.0)
                    .setScale(1.0, 1.0);
                absoluteOdometry = new FtcSparkFunOtos("SparkfunOtos", otosConfig);
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = null; //new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
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
            wheelBaseLength = (24.0 * 15)*TrcUtil.INCHES_PER_MM;
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
            xOdWheelSensorNames = new String[] {"xOdWheelSensor"};
            xOdWheelIndices = new int[] {0};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = new String[] {"yLeftOdWheelSensor", "yRightOdWheelSensor"};
            yOdWheelIndices = new int[] {1, 2};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                    .setOffset(0.0, 0.0, 0.0)
                    .setScale(1.0, 1.0);
                absoluteOdometry = new FtcSparkFunOtos("SparkfunOtos", otosConfig);
            }
            else
            {
                absoluteOdometry = null;
            }
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
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
            webCam1 = new FrontCamParams();
            webCam2 = null; //new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
        }   //CenterStageRobotParams
    }   //class CenterStageRobotParams

    public static class VisionOnlyParams extends FtcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            // Front Camera
            webCam1 = new FrontCamParams();
            // Back Camera
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialRobotParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public DifferentialRobotParams()
        {
            robotName = "DifferentialRobot";
            // Robot Dimensions
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor"};
            driveMotorInverted = new boolean[] {true, false};
            odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            yDrivePosScale = 0.02166184604662450653409090909091;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            yDrivePidCoeffs = new PidCoefficients(0.06, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
        }   //DifferentialRobotParams
    }   //class DifferentialRobotParams

    /**
     * This class contains the Swerve Drive Base Parameters.
     */
    public static class SwerveRobotParams extends FtcSwerveDrive.SwerveInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER = 35.0 * TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public SwerveRobotParams()
        {
            robotName = "SwerveRobot";
            // Robot Dimensions
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = (24.0 * 14)*TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = 16.0;
            // IMU
            imuName = "imu";
            hubLogoDirection = LogoFacingDirection.UP;
            hubUsbDirection = UsbFacingDirection.FORWARD;
            // Drive Motors
            driveMotorType = MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            // Odometry Wheels
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelSensorNames = null;
            xOdWheelIndices = new int[] {FtcRobotDrive.INDEX_RIGHT_BACK};
            xOdWheelXOffsets = new double[] {0.0};
            xOdWheelYOffsets = new double[] {-168.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelSensorNames = null;
            yOdWheelIndices = new int[] {FtcRobotDrive.INDEX_LEFT_FRONT, FtcRobotDrive.INDEX_RIGHT_FRONT};
            yOdWheelXOffsets = new double[] {-144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {144.0 * TrcUtil.INCHES_PER_MM, -12.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            yDrivePosScale = 0.01924724265461924299065420560748;        // in/count
            // Robot Drive Characteristics
            robotMaxVelocity = 23.0;        // inches/sec
            robotMaxAcceleration  = 500.0;  // inches/sec2
            robotMaxTurnRate = 100.0;       // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.95, 0.0, 0.001, 0.0, 0.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = null;
            turnPidCoeffs = new PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new FrontCamParams();
            webCam2 = new BackCamParams();
            limelight = new LimelightParams();
            // Miscellaneous
            blinkinName = "blinkin";
            // Steer Encoders
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.474812, 0.467663, 0.541338, 0.545340};
            steerZerosFilePath = Robot.STEER_ZERO_CAL_FILE;
            // Steer Motors
            steerMotorType = MotorType.CRServo;
            steerMotorNames = new String[] {"lfSteerServo", "rfSteerServo", "lbSteerServo", "rbSteerServo"};
            steerMotorInverted = new boolean[] {true, true, true, true};
            steerMotorPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 1.0;
            // Swerve Modules
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
        }   //SwerveRobotParams
    }   //class SwerveRobotParams

    //
    // Subsystems.
    //

    public static class ElbowParams
    {
    }   //class ElbowParams

    public static class ExtenderParams
    {
        // Elevator Subsystem.
        //
        // Actuator parameters.
        public static final boolean EXTENDER_MOTOR_INVERTED         = false;
        public static final boolean EXTENDER_HAS_LOWER_LIMIT_SWITCH = true;
        public static final boolean EXTENDER_LOWER_LIMIT_INVERTED   = false;
        public static final boolean EXTENDER_HAS_UPPER_LIMIT_SWITCH = false;
        public static final boolean EXTENDER_UPPER_LIMIT_INVERTED   = false;
        public static final boolean EXTENDER_VOLTAGE_COMP_ENABLED   = true;
        public static final double EXTENDER_ENCODER_PPR             = GOBILDA_5203_312_ENCODER_PPR;
        public static final double EXTENDER_PULLEY_DIAMETER         = 1.405;
        public static final double EXTENDER_PULLEY_CIRCUMFERENCE    = Math.PI*EXTENDER_PULLEY_DIAMETER;
        public static final double EXTENDER_INCHES_PER_COUNT        = EXTENDER_PULLEY_CIRCUMFERENCE/EXTENDER_ENCODER_PPR;
        public static final double EXTENDER_POWER_LIMIT             = 1.0;
        public static final double EXTENDER_OFFSET                  = 12.1;             // in inches
        public static final double EXTENDER_MIN_POS                 = EXTENDER_OFFSET;
        public static final double EXTENDER_MAX_POS                 = 22.0;
        public static final double EXTENDER_LOAD_POS                = 12.4;
        public static final double EXTENDER_SAFE_POS                = 15.0; //14.0
        public static final double EXTENDER_HANG_POS                = EXTENDER_MIN_POS;
        public static final double EXTENDER_LEVEL1_POS              = 15.0;
        public static final double EXTENDER_LEVEL2_POS              = 21.5;
        public static final double EXTENDER_LEVEL3_POS              = 21.5;             // Unreachable
        // Power settings.
        public static final double EXTENDER_CAL_POWER               = -0.25;
        // Preset positions.
        public static final double EXTENDER_PRESET_TOLERANCE        = 0.2;
        public static final double[] EXTENDER_PRESETS               = new double[] {
                EXTENDER_LOAD_POS, EXTENDER_LEVEL1_POS, EXTENDER_LEVEL2_POS
//        14.0, 16.0, 20.0, 22.0
        };
        // PID Actuator parameters.
        public static final double EXTENDER_KP                      = 0.6;
        public static final double EXTENDER_KI                      = 0.0;
        public static final double EXTENDER_KD                      = 0.025;
        public static final double EXTENDER_KF                      = 0.0;
        public static final double EXTENDER_TOLERANCE               = 0.5;
        public static final double EXTENDER_IZONE                   = 10.0;
        public static final double EXTENDER_STALL_DETECTION_DELAY   = 0.5;
        public static final double EXTENDER_STALL_DETECTION_TIMEOUT = 0.2;
        public static final double EXTENDER_STALL_ERR_RATE_THRESHOLD= 5.0;
    }   //class ExtenderParams

    public static class WristParams
    {
    }   //class WristParams

    public static class ClimberParams
    {
    }   //class ClimberParams

//    public static class IntakeParams
//    {
//    }   //class IntakeParams
//
//    public static class GrabberParams
//    {
//    }   //class GrabberParams

}   //class RobotParams
