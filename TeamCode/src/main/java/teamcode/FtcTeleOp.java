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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import ftclib.drivebase.FtcSwerveDrive;
import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import teamcode.params.GameParams;
import teamcode.params.RobotParams;
import teamcode.subsystems.AuxClimber;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Wrist;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.timer.TrcTimer;

/**
 * This class contains the TeleOp Mode program.
 */
@TeleOp(name="FtcTeleOp", group="Ftc3543")
public class FtcTeleOp extends FtcOpMode
{
    private final String moduleName = getClass().getSimpleName();

    protected Robot robot;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    private double drivePowerScale;
    private double turnPowerScale;
    private boolean driverAltFunc = false;
    private boolean operatorAltFunc = false;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
    private double elbowPrevPower = 0.0;
    private double extenderPrevPower = 0.0;

    private Robot.GamePieceType objectType;
    private Robot.ScoreHeight scoreHeight = Robot.ScoreHeight.HIGH;

    //
    // Implements FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize robot object.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Open trace log.
        //
        if (RobotParams.Preferences.useTraceLog)
        {
            String filePrefix = Robot.matchInfo != null?
                String.format(Locale.US, "%s%02d_TeleOp", Robot.matchInfo.matchType, Robot.matchInfo.matchNumber):
                "Unknown_TeleOp";
            TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, filePrefix);
        }
        //
        // Create and initialize Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        driverGamepad.setButtonEventHandler(this::driverButtonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
        driverGamepad.setLeftStickInverted(false, true);
        driverGamepad.setRightStickInverted(false, true);
        operatorGamepad.setLeftStickInverted(false, true);
        operatorGamepad.setRightStickInverted(false, true);
        drivePowerScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
        turnPowerScale = RobotParams.Robot.TURN_NORMAL_SCALE;
        setDriveOrientation(RobotParams.Robot.DRIVE_ORIENTATION);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station is pressed. Typically, you put code that will prepare the robot for start of
     * competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(
            moduleName, "***** Starting TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        robot.dashboard.clearDisplay();
        //
        // Tell robot object opmode is about to start so it can do the necessary start initialization for the mode.
        //
        robot.startMode(nextMode);
        //
        // Enable AprilTag vision for re-localization.
        //
        if (robot.vision != null)
        {
            if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling Limelight AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(0, true);
            }
            else if (robot.vision.aprilTagVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling WebCam AprilTagVision.");
                robot.vision.setAprilTagVisionEnabled(true);
            }
        }
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Tell robot object opmode is about to stop so it can do the necessary cleanup for the mode.
        //
        robot.stopMode(prevMode);
        printPerformanceMetrics();
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");

        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.closeTraceLog();
        }
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            if (robot.robotDrive != null)
            {
                // We are trying to re-localize the robot and vision hasn't seen AprilTag yet.
                if (relocalizing)
                {
                    if (robotFieldPose == null)
                    {
                        robotFieldPose = robot.vision.getRobotFieldPose();
                    }
                }
                else
                {
                    double[] inputs = driverGamepad.getDriveInputs(
                        RobotParams.Robot.DRIVE_MODE, true, drivePowerScale, turnPowerScale);

                    if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        robot.robotDrive.driveBase.holonomicDrive(
                            null, inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveGyroAngle());
                    }
                    else
                    {
                        robot.robotDrive.driveBase.arcadeDrive(inputs[1], inputs[2]);
                    }
                    robot.dashboard.displayPrintf(
                        1, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                        inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveOrientation());
                }
            }
            //
            // Other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                // Analog control of subsystems.
                //elbow subsystem
                if (robot.elbow != null)
                {
                    double elbowPower = operatorGamepad.getRightStickY(true) * Elbow.Params.POWER_LIMIT;
                    if (elbowPower != elbowPrevPower)
                    {
                        if (operatorAltFunc)
                        {
                            robot.elbow.setPower(elbowPower);
                        }
                        else
                        {
                            robot.elbow.setPidPower(elbowPower, Elbow.Params.MIN_POS, Elbow.Params.MAX_POS, true);
                        }
                        elbowPrevPower = elbowPower;
                    }
                }

                //extender arm subsystem
                if (robot.extender != null)
                {
                    double extenderPower = operatorGamepad.getLeftStickY(true) * Extender.Params.POWER_LIMIT;
                    if (extenderPower != extenderPrevPower)
                    {
                        if (operatorAltFunc)
                        {
                            robot.extender.setPower(extenderPower);
                        }
                        else
                        {
                            robot.extender.setPidPower(
                                extenderPower, Extender.Params.MIN_POS, Extender.Params.MAX_POS, true);
                        }
                        extenderPrevPower = extenderPower;
                    }
                }
            }
            // Display subsystem status.
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus(2);
            }
        }
    }   //periodic

    /**
     * This method sets the drive orientation mode and updates the LED to indicate so.
     *
     * @param orientation specifies the drive orientation (FIELD, ROBOT, INVERTED).
     */
    public void setDriveOrientation(TrcDriveBase.DriveOrientation orientation)
    {
        if (robot.robotDrive != null)
        {
            robot.globalTracer.traceInfo(moduleName, "driveOrientation=" + orientation);
            robot.robotDrive.driveBase.setDriveOrientation(
                orientation, orientation == TrcDriveBase.DriveOrientation.FIELD);
            if (robot.ledIndicator != null)
            {
                robot.ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void driverButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Driver: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
                // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
                if (driverAltFunc)
                {
                    if (pressed && robot.robotDrive != null)
                    {
                        if (robot.robotDrive.driveBase.isGyroAssistEnabled())
                        {
                            // Disable GyroAssist drive.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                        }
                        else
                        {
                            // Enable GyroAssist drive.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                        }
                    }
                }
                else
                {
                    if (pressed && robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        if (robot.robotDrive.driveBase.getDriveOrientation() != TrcDriveBase.DriveOrientation.FIELD)
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling FIELD mode.");
                            setDriveOrientation(TrcDriveBase.DriveOrientation.FIELD);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling ROBOT mode.");
                            setDriveOrientation(TrcDriveBase.DriveOrientation.ROBOT);
                        }
                    }
                }
                break;

            case B:
                if (robot.extenderArm != null && pressed)
                {
                    robot.extenderArm.retract(null);
                }
                break;

            case X:
                if (robot.extenderArm != null && pressed &&
                    robot.pickupFromGroundTask != null && robot.pickupSpecimenTask != null)
                {
                    if (!driverAltFunc)
                    {
                        if (!robot.pickupFromGroundTask.isActive())
                        {
                            objectType = Robot.GamePieceType.SAMPLE;
                            robot.pickupFromGroundTask.autoPickupFromGround(
                                Vision.SampleType.AnySample, // Logic to decide which one later
                                null);
                        }
                        else
                        {
                            robot.pickupFromGroundTask.cancel();
                        }
                    }
                    else
                    {
                        if (!robot.pickupSpecimenTask.isActive())
                        {
                            objectType = Robot.GamePieceType.SPECIMEN;
                            robot.pickupSpecimenTask.autoPickupSpecimen(null, null);
                        }
                        else
                        {
                            robot.pickupSpecimenTask.cancel();
                        }
                    }
                }
                break;

            case Y:
                if (robot.extenderArm != null && pressed)
                {
                    // Due to the geometry of the intake,
                    // a sample will not be detected by the color,
                    // but a specimen will be
                    if (objectType == Robot.GamePieceType.SPECIMEN)
                    {
                        if (robot.scoreChamberTask != null)
                        {
                            if (!robot.scoreChamberTask.isActive())
                            {
                                robot.scoreChamberTask.autoScoreChamber(null, null, scoreHeight, !driverAltFunc, null);
                            }
                            else
                            {
                                robot.scoreChamberTask.cancel();
                            }
                        }
                    }
                    else
                    {
                        if (robot.scoreBasketTask != null)
                        {
                            if (!robot.scoreBasketTask.isActive())
                            {
                                robot.scoreBasketTask.autoScoreBasket(null, scoreHeight, !driverAltFunc, null);
                            }
                            else
                            {
                                robot.scoreChamberTask.cancel();
                            }
                        }
                    }
                }
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + pressed);
                driverAltFunc = pressed;
                break;

            case RightBumper:
                // Press and hold for slow drive.
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
                    drivePowerScale = RobotParams.Robot.DRIVE_SLOW_SCALE;
                    turnPowerScale = RobotParams.Robot.TURN_SLOW_SCALE;
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
                    drivePowerScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
                    turnPowerScale = RobotParams.Robot.TURN_NORMAL_SCALE;
                }
                break;

            case DpadUp:
                if (pressed)
                {
                    if (scoreHeight == Robot.ScoreHeight.HIGH)
                    {
                        scoreHeight = Robot.ScoreHeight.LOW;
                    }
                    else
                    {
                        scoreHeight = Robot.ScoreHeight.HIGH;
                    }
                }
                break;

            case DpadDown:
                break;

            case DpadLeft:
                if (robot.grabber != null && pressed)
                {
                    if (!robot.grabber.isAutoActive())
                    {
                        robot.grabber.autoIntake(null, 0.0, null);
                    }
                    else
                    {
                        robot.grabber.cancel();
                    }
                }
                break;

            case DpadRight:
                if (robot.grabber != null && pressed)
                {
                    if (!robot.grabber.isAutoActive())
                    {
                        robot.grabber.autoDump(null, 0.0, null);
                    }
                    else
                    {
                        robot.grabber.cancel();
                    }
                }
                break;

            case Back:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate();
                    if (robot.robotDrive != null && robot.robotDrive instanceof FtcSwerveDrive)
                    {
                        // Drive base is a Swerve Drive, align all steering wheels forward.
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set SteerAngle to zero.");
                        ((FtcSwerveDrive) robot.robotDrive).setSteerAngle(0.0, false, false);
                    }
                }
                break;

            case Start:
                if (!driverAltFunc)
                {
                    if (robot.vision != null &&
                        (robot.vision.isLimelightVisionEnabled() || robot.vision.isAprilTagVisionEnabled()) &&
                        robot.robotDrive != null)
                    {
                        // On press of the button, we will start looking for AprilTag for re-localization.
                        // On release of the button, we will set the robot's field location if we found the AprilTag.
                        relocalizing = pressed;
                        if (!pressed)
                        {
                            if (robotFieldPose != null)
                            {
                                // Vision found an AprilTag, set the new robot field location.
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Finish re-localizing: pose=" + robotFieldPose);
                                robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                                robotFieldPose = null;
                            }
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                        }
                    }
                }
                else
                {
                    if (pressed)
                    {
                        robot.robotDrive.driveBase.setFieldPosition(GameParams.STARTPOSE_RED_NET_ZONE);
                    }
                }
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    public void operatorButtonEvent(FtcGamepad.ButtonType button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Operator: %s=%s", button, pressed? "Pressed": "Released");

        switch (button)
        {
            case A:
                if (robot.wrist != null && pressed)
                {
                    robot.wrist.setPosition(Wrist.Params.MAX_POS);
                }
                break;

            case B:
                if (robot.wrist != null && pressed)
                {
                    robot.wrist.setPosition(0.575); // ??? What is this position???
                }
                break;

            case X:
                if (robot.wrist != null && pressed)
                {
                    robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS);
                }
                break;

            case Y:
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + pressed);
                operatorAltFunc = pressed;
                break;

            case RightBumper:
                if (robot.extenderArm != null && pressed)
                {
                    robot.extenderArm.retract(null);
                }
                break;

            case DpadUp:
                //auxiliary climber subsystem goes up
                if (robot.auxClimber != null && pressed)
                {
                    robot.auxClimber.setPosition(AuxClimber.Params.MAX_POS);
                }
                break;

            case DpadDown:
                //auxiliary climber subsystem goes down
                if (robot.auxClimber != null && pressed)
                {
                   robot.auxClimber.setPosition(AuxClimber.Params.MIN_POS);
                }
                break;

            case DpadLeft:
                if (robot.grabber != null)
                {
                    if (pressed)
                    {
                        if (operatorAltFunc)
                        {
                            // This is manual override in case the sensor is not working, just turn it ON.
                            robot.grabber.intake(null, 0.0, null);
                        }
                        else
                        {
                            robot.grabber.autoIntake(null, 0.0, null);
                        }
                    }
                    else
                    {
                        if (robot.grabber.isAutoActive())
                        {
                            robot.grabber.cancel();
                        }
                        else
                        {
                            // This is manual override in case the sensor is not working, just turn it OFF.
                            robot.grabber.stop(null);
                        }
                    }
                }
                break;

            case DpadRight:
                if (robot.grabber != null)
                {
                    if (pressed)
                    {
                        if (operatorAltFunc)
                        {
                            // This is manual override in case the sensor is not working, just turn it ON.
                            robot.grabber.dump(null, 0.0, null);
                        }
                        else
                        {
                            robot.grabber.autoDump(null, 0.0, null);
                        }
                    }
                    else
                    {
                        if (robot.grabber.isAutoActive())
                        {
                            robot.grabber.cancel();
                        }
                        else
                        {
                            // This is manual override in case the sensor is not working, just turn it OFF.
                            robot.grabber.stop(null);
                        }
                    }
                }
                break;

            case Back:
                if (pressed)
                {
                    // Zero calibrate all subsystems (arm, elevator and turret).
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrate pressed.");
                    robot.cancelAll();
                    robot.zeroCalibrate(moduleName);
                }
                break;

            case Start:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
