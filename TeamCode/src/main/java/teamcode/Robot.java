/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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
import teamcode.autotasks.TaskExtenderArm;
import teamcode.subsystems.AuxClimber;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Intake;
import teamcode.subsystems.RobotBase;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Wrist;
import teamcode.vision.Vision;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcDigitalInput;
import trclib.subsystem.TrcIntake;
import trclib.subsystem.TrcServoGrabber;
import trclib.timer.TrcTimer;

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
    public TrcMotor auxClimber;
    public TrcIntake intake;
    public TrcServoGrabber grabber;

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
        if (RobotParams.Preferences.robotType != RobotParams.RobotType.VisionOnly)
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
                    extenderArm = new TaskExtenderArm("ExtenderArm", this);
                }

                if (RobotParams.Preferences.useAuxClimber)
                {
                    auxClimber = new AuxClimber().getClimber();
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake().getIntake();
                }

                if (RobotParams.Preferences.useGrabber)
                {
                    grabber = new Grabber().getGrabber();
                }
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
                if (elbow != null)
                {
                    dashboard.displayPrintf(
                        lineNum++,
                        "Elbow: power=%.3f,pos=%.1f/%.1f,limitSw=%s",
                        elbow.getPower(), elbow.getPosition(), elbow.getPidTarget(), elbow.isLowerLimitSwitchActive());
                }

                if (extender != null)
                {
                    dashboard.displayPrintf(
                        lineNum++,
                        "Extender: power=%.3f,pos=%.1f/%.1f,limitSw=%s",
                        extender.getPower(), extender.getPosition(), extender.getPidTarget(),
                        extender.isLowerLimitSwitchActive());
                }

                if (wrist != null)
                {
                    dashboard.displayPrintf(lineNum++, "Wrist: pos=%.3f", wrist.getPosition());
                }

                if (auxClimber != null)
                {
                    dashboard.displayPrintf(
                        lineNum++,
                        "AuxClimber: power=%.3f,pos=%.1f/%.1f,limitSw=%s",
                        auxClimber.getPower(), auxClimber.getPosition(), auxClimber.getPidTarget(),
                        auxClimber.isLowerLimitSwitchActive());
                }

                if (intake != null)
                {
                    dashboard.displayPrintf(lineNum++, "Intake: power=%.3f", intake.getPower());
                }

                if (grabber != null)
                {
                    if (RobotParams.GrabberParams.USE_ANALOG_SENSOR)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Grabber: pos=%.3f,hasObject=%s,sensorValue=%.3f,autoActive=%s",
                            grabber.getPosition(), grabber.hasObject(), grabber.getSensorValue(),
                            grabber.isAutoAssistActive());
                    }
                    else if (RobotParams.GrabberParams.USE_DIGITAL_SENSOR)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Grabber: pos=%.3f,hasObject=%s,sensorState=%s,autoActive=%s",
                            grabber.getPosition(), grabber.hasObject(), grabber.getSensorState(),
                            grabber.isAutoAssistActive());
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

        if (extenderArm != null) extenderArm.cancel();
        if (elbow != null) elbow.cancel();
        if (extender != null) extender.cancel();
        if (wrist != null) wrist.cancel();
        if (auxClimber != null) auxClimber.cancel();
        if (intake != null) intake.cancel();
        if (grabber != null) grabber.cancel();
        if (robotDrive != null) robotDrive.cancel();
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (elbow != null)
        {
            elbow.zeroCalibrate(owner, RobotParams.ElbowParams.ZERO_CAL_POWER);
        }

        if (extender != null)
        {
            extender.zeroCalibrate(owner, RobotParams.ExtenderParams.ZERO_CAL_POWER);
        }

        if (auxClimber != null)
        {
            auxClimber.zeroCalibrate(owner, RobotParams.ClimberParams.ZERO_CAL_POWER);
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
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
//        robotDrive.driveBase.setFieldPosition(
//                adjustPoseByAlliance(
//                        autoChoices.startPos == FtcAuto.StartPos.BASKET?
//                                RobotParams.STARTPOS_BASKET: RobotParams.STARTPOS_BLUE_BACKSTAGE,
//                        autoChoices.alliance, false));
    }   //setRobotStartPosition

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

    public TrcPose2D adjustPoseByAlliance(
            double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = -newPose.y;
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.Game.FULL_TILE_INCHES;
            newPose.y *= RobotParams.Game.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance
}   //class Robot
