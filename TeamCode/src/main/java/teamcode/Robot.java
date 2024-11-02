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

package teamcode;

import androidx.annotation.NonNull;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.driverio.FtcDashboard;
import ftclib.driverio.FtcMatchInfo;
import ftclib.robotcore.FtcOpMode;
import ftclib.sensor.FtcRobotBattery;
import teamcode.autotasks.TaskAutoPickupFromGround;
import teamcode.autotasks.TaskAutoPickupSpecimen;
import teamcode.autotasks.TaskAutoScoreBasket;
import teamcode.autotasks.TaskAutoScoreChamber;
import teamcode.autotasks.TaskExtenderArm;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.RobotBase;
import teamcode.subsystems.Wrist;
import teamcode.subsystems.Vision;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.subsystem.TrcMotorGrabber;
import trclib.subsystem.TrcServoGrabber;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private final String moduleName = getClass().getSimpleName();
    // Global objects.
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    private static double nextStatusUpdateTime = 0.0;
    public static Vision.SampleType sampleType = Vision.SampleType.AnySample;
    // Robot Drive.
    public FtcRobotDrive.RobotInfo robotInfo;
    public FtcRobotDrive robotDrive;
    // Vision subsystems.
    public Vision vision;
    // Sensors and indicators.
    public LEDIndicator ledIndicator;
    public FtcRobotBattery battery;
    // Subsystems.
    public TrcMotor elbow;
    public TrcMotor extender;
    public TrcServo wrist;
    public TaskExtenderArm extenderArm;
    public Grabber grabber;
    // Autotasks.
    public TaskAutoPickupFromGround pickupFromGroundTask;
    public TaskAutoPickupSpecimen pickupSpecimenTask;
    public TaskAutoScoreBasket scoreBasketTask;
    public TaskAutoScoreChamber scoreChamberTask;

    public enum GamePieceType
    {
        SPECIMEN,
        SAMPLE
    }   //enum GamePieceType

    public enum ScoreHeight
    {
        HIGH,
        LOW
    }   //enum ScoreHeight

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        // Initialize global objects.
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        nextStatusUpdateTime = TrcTimer.getCurrentTime();
        speak("Init starting");
        // Create and initialize Robot Base.
        RobotBase robotBase = new RobotBase();
        robotInfo = robotBase.getRobotInfo();
        robotDrive = robotBase.getRobotDrive();
        // Create and initialize vision subsystems.
        if (RobotParams.Preferences.useVision &&
            (RobotParams.Preferences.tuneColorBlobVision ||
             RobotParams.Preferences.useAprilTagVision ||
             RobotParams.Preferences.useColorBlobVision ||
             RobotParams.Preferences.useLimelightVision))
        {
            vision = new Vision(this);
        }
        // If robotType is VisionOnly, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != RobotBase.RobotType.VisionOnly)
        {
            // Create and initialize sensors and indicators.
            if (robotInfo.indicatorName != null)
            {
                ledIndicator = new LEDIndicator(robotInfo.indicatorName);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
//                TrcMotor elbow = null;
//                TrcMotor extender = null;
//                TrcServo wrist = null;
//
                if (RobotParams.Preferences.useElbow)
                {
                    elbow = new Elbow(this).getMotor();
                }

                if (RobotParams.Preferences.useExtender)
                {
                    extender = new Extender().getMotor();
                }

                if (RobotParams.Preferences.useWrist)
                {
                    wrist = new Wrist().getServo();
                }

                if (RobotParams.Preferences.useElbow &&
                    RobotParams.Preferences.useExtender &&
                    RobotParams.Preferences.useWrist)
                {
                    extenderArm = new TaskExtenderArm("ExtenderArm", elbow, extender, wrist);
                }

                if (RobotParams.Preferences.useMotorGrabber || RobotParams.Preferences.useServoGrabber)
                {
                    grabber = new Grabber(this);
                }
                // Zero calibrate all subsystems only at init time.
                zeroCalibrate();
                // Create autotasks.
                pickupFromGroundTask = new TaskAutoPickupFromGround("pickupFromGroundTask", this);
                pickupSpecimenTask = new TaskAutoPickupSpecimen("pickupSpecimenTask", this);
                scoreBasketTask = new TaskAutoScoreBasket("scoreBasketTask", this);
                scoreChamberTask = new TaskAutoScoreChamber("scoreChamberTask", this);
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return robotInfo != null? robotInfo.robotName: RobotParams.Robot.ROBOT_CODEBASE;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (vision != null)
        {
            // Enable the correct sample type vision according to Alliance color from AutoChoices or if in standalone
            // TeleOp (no prior auto run), enable vision for any sample.
            vision.setSampleVisionEnabled(sampleType, true);
        }

        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            vision.setCameraStreamEnabled(false);
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.redSampleVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RedSampleVision.");
                vision.setSampleVisionEnabled(Vision.SampleType.RedSample, false);
            }

            if (vision.blueSampleVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling BlueSampleVision.");
                vision.setSampleVisionEnabled(Vision.SampleType.BlueSample, false);
            }

            if (vision.yellowSampleVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling YellowSampleVision.");
                vision.setSampleVisionEnabled(Vision.SampleType.YellowSample, false);
            }

            if (vision.limelightVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling LimelightVision.");
                vision.setLimelightVisionEnabled(0, false);
            }

            vision.close();
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method update all subsystem status on the dashboard.
     *
     * @param startLineNum specifies the first Dashboard line for printing status.
     */
    public void updateStatus(int startLineNum)
    {
        double currTime = TrcTimer.getCurrentTime();
        if (currTime > nextStatusUpdateTime)
        {
            int lineNum = startLineNum;
            nextStatusUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
            if (robotDrive != null)
            {
                dashboard.displayPrintf(lineNum++, "DriveBase: Pose=%s", robotDrive.driveBase.getFieldPosition());
            }
            //
            // Display other subsystem status here.
            //
            if (RobotParams.Preferences.showSubsystems)
            {
                if (extenderArm != null)
                {
                    if (extenderArm.elbow != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++,
                            "Elbow: power=%.3f,pos=%.1f/%.1f,limitSw=%s",
                            extenderArm.elbow.getPower(), extenderArm.elbow.getPosition(),
                            extenderArm.elbow.getPidTarget(), extenderArm.elbow.isLowerLimitSwitchActive());
                    }

                    if (extenderArm.extender != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++,
                            "Extender: power=%.3f,pos=%.1f/%.1f,limitSw=%s",
                            extenderArm.extender.getPower(), extenderArm.extender.getPosition(),
                            extenderArm.extender.getPidTarget(), extenderArm.extender.isLowerLimitSwitchActive());
                    }

                    if (extenderArm.wrist != null)
                    {
                        dashboard.displayPrintf(lineNum++, "Wrist: pos=%.3f", extenderArm.wrist.getPosition());
                    }
                }

                if (grabber != null)
                {
                    TrcMotorGrabber motorGrabber = grabber.getMotorGrabber();
                    TrcServoGrabber servoGrabber = grabber.getServoGrabber();
                    if (motorGrabber != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++,
                            "MotorGrabber: power=%.3f,hasObject=%s,sensorDist=%.3f,sensorHue=%.3f,sample=%s," +
                            "autoActive=%s",
                            motorGrabber.getPower(), grabber.hasObject(), grabber.getSensorDistance(),
                            grabber.getSensorHue(), grabber.getSampleType(), grabber.isAutoActive());
                    }
                    else
                    {
                        dashboard.displayPrintf(
                            lineNum++,
                            "ServoGrabber: pos=%.3f,hasObject=%s,sensorDist=%.3f,sensorHue=%.3f,sample=%s," +
                            "autoActive=%s",
                            servoGrabber.getPosition(), grabber.hasObject(), grabber.getSensorDistance(),
                            grabber.getSensorHue(), grabber.getSampleType(), grabber.isAutoActive());
                    }
                }
            }
        }
    }   //updateStatus

    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");
        // Cancel subsystems.
        if (extenderArm != null) extenderArm.cancel();
        if (grabber != null) grabber.cancel();
        if (robotDrive != null) robotDrive.cancel();
        // Cancel auto tasks.
        if (pickupFromGroundTask != null) pickupFromGroundTask.cancel();
        if (pickupSpecimenTask != null) pickupSpecimenTask.cancel();
        if (scoreBasketTask != null) scoreBasketTask.cancel();
        if (scoreChamberTask != null) scoreChamberTask.cancel();
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (extenderArm != null)
        {
            extenderArm.zeroCalibrate(owner);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method calls vision to detect the specified sample and return the sample pose.
     *
     * @param sampleType specifies the sample type to look for.
     * @param sampleGroundOffset specifies the object ground offset above the floor.
     * @param logInfo specifies true to log detected info into tracelog, false otherwise.
     * @return detected sample pose, null if vision is busy or no sample detected.
     */
    public TrcPose2D getDetectedSamplePose(Vision.SampleType sampleType, double sampleGroundOffset, boolean logInfo)
    {
        TrcPose2D samplePose = null;

        if (vision != null && vision.isSampleVisionEnabled(sampleType))
        {
            TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo =
                vision.getDetectedSample(sampleType, sampleGroundOffset, -1);
            if (sampleInfo != null)
            {
                samplePose = robotInfo.webCam1.camPose.toPose2D().addRelativePose(sampleInfo.objPose);
                double extenderLen = TrcUtil.magnitude(samplePose.x, samplePose.y) - Extender.Params.PIVOT_Y_OFFSET;
                samplePose.angle = Math.toDegrees(Math.atan(samplePose.x/samplePose.y));
                if (ledIndicator != null && extenderLen < Extender.Params.MAX_POS)
                {
                    ledIndicator.setDetectedPattern(sampleInfo.detectedObj.label);
                }
                if (logInfo)
                {
                    globalTracer.traceInfo(
                        moduleName, "detectedSamplePose=%s, adjustedSamplePose=%s", sampleInfo.objPose, samplePose);
                }
            }
        }

        return samplePose;
    }   //getDetectedSamplePose

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        robotDrive.driveBase.setFieldPosition(
            adjustPoseByAlliance(
                autoChoices.startPos == FtcAuto.StartPos.NET_ZONE?
                    RobotParams.Game.STARTPOSE_RED_NET_ZONE: RobotParams.Game.STARTPOSE_RED_OBSERVATION_ZONE,
                autoChoices.alliance, false));
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in the specified unit.
     * @param y specifies y position in the red alliance in the specified unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if x and y are in tile unit, false if in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.BLUE_ALLIANCE)
        {
            // Translate blue alliance pose to red alliance pose.
            if (RobotParams.Game.fieldIsMirrored)
            {
                // Mirrored field.
                double angleDelta = (newPose.angle - 90.0)*2.0;
                newPose.angle -= angleDelta;
                newPose.y = -newPose.y;
            }
            else
            {
                // Symmetrical field.
                newPose.x = -newPose.x;
                newPose.y = -newPose.y;
                newPose.angle = (newPose.angle + 180.0) % 360.0;
            }
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.Field.FULL_TILE_INCHES;
            newPose.y *= RobotParams.Field.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param x specifies x position in the red alliance in tile unit.
     * @param y specifies y position in the red alliance in tile unit.
     * @param heading specifies heading in the red alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, false);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the red alliance in the specified unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false in inches.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the red alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, false);
    }   //adjustPoseByAlliance

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
