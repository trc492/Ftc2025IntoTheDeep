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

package teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import ftclib.drivebase.FtcMecanumDrive;
import ftclib.drivebase.FtcRobotDrive;
import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcPinpointOdometry;
import ftclib.sensor.FtcSparkFunOtos;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class RobotBase
{
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
     * This class contains the IntoTheDeep Robot Parameters.
     */
    public static class IntoTheDeepRobotParams extends FtcRobotDrive.RobotInfo
    {
        // Optii Odometry Wheel
        private static final double ODWHEEL_DIAMETER_MM = 35.0;
        private static final double ODWHEEL_DIAMETER = ODWHEEL_DIAMETER_MM*TrcUtil.INCHES_PER_MM;
        private static final double ODWHEEL_CPR = 4096.0;

        public IntoTheDeepRobotParams()
        {
            robotName = "IntoTheDeepRobot";
            // Robot Dimensions (measured from CAD model if possible)
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = 360.0 * TrcUtil.INCHES_PER_MM;
            wheelBaseWidth = (312.0 + 103.623) * TrcUtil.INCHES_PER_MM;
            // IMU (not used if using AbsoluteOdometry).
            imuName = null;
            hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;   // Control Hub orientation
            hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;        // Control Hub orientation
            // Drive Motors
            driveMotorType = FtcMotorActuator.MotorType.DcMotor;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorInverted = new boolean[] {true, false, true, false};
            odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            // Odometry Wheels (Offset from wheel base center)
            odWheelXScale = odWheelYScale = Math.PI * ODWHEEL_DIAMETER / ODWHEEL_CPR;
            xOdWheelXOffsets = new double[] {4.0 * TrcUtil.INCHES_PER_MM};
            xOdWheelYOffsets = new double[] {-132.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelXOffsets = new double[] {-156.0 * TrcUtil.INCHES_PER_MM, 156.0 * TrcUtil.INCHES_PER_MM};
            yOdWheelYOffsets = new double[] {-4.0 * TrcUtil.INCHES_PER_MM, -4.0 * TrcUtil.INCHES_PER_MM};
            // Absolute Odometry
            if (odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
            {
                if (RobotParams.Preferences.usePinpointOdometry)
                {
                    FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                        .setPodOffsets(-156.0, -131.4)  // Offsets from robot center in mm
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
            // Robot Drive Characteristics
            robotMaxVelocity = 80.0;        // inches/sec
            robotMaxAcceleration  = 350.0;  // inches/sec2
            robotMaxTurnRate = 80.0;        // degrees/sec
            profiledMaxVelocity = robotMaxVelocity;
            profiledMaxAcceleration = robotMaxAcceleration;
            profiledMaxTurnRate = robotMaxTurnRate;
            // DriveBase PID Parameters
            drivePidTolerance = 1.5;
            turnPidTolerance = 1.5;
            xDrivePidCoeffs = new TrcPidController.PidCoefficients(0.072, 0.001, 0.0065, 0.0, 2.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new TrcPidController.PidCoefficients(0.037, 0.002, 0.0035, 0.0, 2.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new TrcPidController.PidCoefficients(0.032, 0.1, 0.0025, 0.0, 5.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new Vision.SampleCamParams();
            webCam2 = null;
            limelight = new Vision.LimelightParams();
            // Miscellaneous
            indicatorName = RobotParams.Preferences.useBlinkinLED? "blinkin":
                            RobotParams.Preferences.useGobildaLED? "gobildaLED": null;
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
            hubLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            hubUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            // Drive Motors
            driveMotorType = FtcMotorActuator.MotorType.DcMotor;
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
            xDrivePidCoeffs = new TrcPidController.PidCoefficients(0.095, 0.0, 0.006, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            xDriveMaxPidRampRate = null;
            yDrivePidCoeffs = new TrcPidController.PidCoefficients(0.025, 0.0, 0.002, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            yDriveMaxPidRampRate = null;
            turnPidCoeffs = new TrcPidController.PidCoefficients(0.013, 0.0, 0.0011, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            turnMaxPidRampRate = null;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 6.0;
            velPidCoeffs = new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/profiledMaxVelocity, 0.0);
            // Vision
            webCam1 = new Vision.SampleCamParams();
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
            webCam1 = new Vision.SampleCamParams();
            webCam2 = null;
            limelight = new Vision.LimelightParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    private final FtcRobotDrive.RobotInfo robotInfo;
    private final FtcRobotDrive robotDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotBase()
    {
        switch (RobotParams.Preferences.robotType)
        {
            case IntoTheDeepRobot:
                robotInfo = new IntoTheDeepRobotParams();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcMecanumDrive(robotInfo): null;
                break;

            case CenterStageRobot:
                robotInfo = new CenterStageRobotParams();
                robotDrive = RobotParams.Preferences.useDriveBase? new FtcMecanumDrive(robotInfo): null;
                break;

            case VisionOnly:
                robotInfo = new VisionOnlyParams();
                robotDrive = null;
                break;

            default:
                robotInfo = null;
                robotDrive = null;
                break;
        }
    }   //RobotBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FtcRobotDrive.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FtcRobotDrive getRobotDrive()
    {
        return robotDrive;
    }   //getRobotDrive

}   //class RobotBase
