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

import ftclib.driverio.FtcGamepad;
import ftclib.robotcore.FtcOpMode;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.Wrist;
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
    private Robot.ScoreHeight scoreHeight = Robot.ScoreHeight.HIGH;
    private double elbowPos, extenderPos;

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
        driverGamepad.setLeftStickInverted(false, true);
        driverGamepad.setRightStickInverted(false, true);

        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        operatorGamepad.setButtonEventHandler(this::operatorButtonEvent);
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
            robot.getDetectedSamplePose(Robot.sampleType, 0.0, false);
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
                if (robot.elbow != null)
                {
                    double elbowPower = operatorGamepad.getRightStickY(true) * Elbow.Params.POWER_LIMIT;

                    elbowPos = robot.elbow.getPosition();
                    extenderPos = robot.extender.getPosition();
                    double extenderLimit = Extender.Params.MAX_POS - Math.cos(Math.toRadians(elbowPos)) * 6.0;
                    if (elbowPos < 60.0 && extenderPos > extenderLimit)
                    {
                        robot.extender.setPosition(extenderLimit);
                        if (elbowPos < Elbow.Params.SAFE_POS)
                        {
                            robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS);
                        }
                    }

                    if (elbowPower != elbowPrevPower)
                    {

//                        if (robot.elbow.getPosition() < Elbow.Params.SAFE_POS && robot.extender.getPosition() > Extender.Params.MAX_POS - 6.0)
//                        {
////                            robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS);
//                            if (robot.wrist.getPosition() == Wrist.Params.GROUND_PICKUP_POS) {
//                                robot.extender.setPosition(Extender.Params.MAX_POS - 6.0);
//                            }
//
//                        }
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
                            double adjust = Math.cos(Math.toRadians(robot.elbow.getPosition())) * 6.0;
                            robot.extender.setPidPower(
//                                extenderPower, Extender.Params.MIN_POS, Extender.Params.MAX_POS - (robot.elbow.getPosition() < Elbow.Params.SAFE_POS ? 6.0 : 0.0 ), true);
                                extenderPower, Extender.Params.MIN_POS, Extender.Params.MAX_POS - (adjust >= 0? adjust: 0.0), true);
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
                if (robot.robotDrive != null && pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotDrive.driveBase.isGyroAssistEnabled())
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Disabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling GyroAssist.");
                            robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                        }
                    }
                    else if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                    {
                        // Toggle between field or robot oriented driving, only applicable for holonomic drive base.
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
                if (robot.scoreChamberTask != null && pressed)
                {
                    if (!robot.scoreChamberTask.isActive())
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Auto score chamber (scoreHeight=%s).", scoreHeight);
                        robot.scoreChamberTask.autoScoreChamber(scoreHeight, false,null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto score chamber.");
                        robot.scoreChamberTask.cancel();
                    }
                }
                break;

            case X:
                if (robot.scoreBasketTask != null && pressed)
                {
                    if (!robot.scoreBasketTask.isActive())
                    {
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Auto score basket (scoreHeight=%s).", scoreHeight);
                        robot.scoreBasketTask.autoScoreBasket(null, scoreHeight, !driverAltFunc, null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto score basket.");
                        robot.scoreChamberTask.cancel();
                    }
                }
                break;

            case Y:
                if (driverAltFunc && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel all.");
                    robot.cancelAll();
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
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set score height low.");
                        scoreHeight = Robot.ScoreHeight.LOW;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setDetectedPattern(LEDIndicator.SCORE_HEIGHT_HIGH);
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set score height high.");
                        scoreHeight = Robot.ScoreHeight.HIGH;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setDetectedPattern(LEDIndicator.SCORE_HEIGHT_LOW);
                        }
                    }
                }
                break;

            case DpadDown:
                // TODO: press once climb low rung, press again climb high rung.
                break;

            case DpadLeft:
                if (robot.pickupFromGroundTask != null && pressed)
                {
                    if (!robot.pickupFromGroundTask.isActive())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto pickup from ground.");
                        robot.pickupFromGroundTask.autoPickupFromGround(Robot.sampleType, true, false, null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto pickup from ground.");
                        robot.pickupFromGroundTask.cancel();
                    }
                }
                break;

            case DpadRight:
                if (robot.pickupSpecimenTask != null && pressed)
                {
                    if (!robot.pickupSpecimenTask.isActive())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto pickup specimen.");
                        robot.pickupSpecimenTask.autoPickupSpecimen(null, !driverAltFunc, null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto pickup specimen.");
                        robot.pickupSpecimenTask.cancel();
                    }
                }
                break;

            case Back:
                if (pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrating.");
                    robot.cancelAll();
                    robot.zeroCalibrate();
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
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set start position to RED_NET_ZONE.");
                        robot.robotDrive.driveBase.setFieldPosition(RobotParams.Game.STARTPOSE_RED_NET_ZONE);
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
                if (robot.wrist != null && pressed &&
                    (elbowPos > Elbow.Params.SAFE_POS || extenderPos < Extender.Params.MAX_POS - 6.0))
                {
                    robot.wrist.setPosition(Wrist.Params.HIGH_BASKET_SCORE_POS);
                }
                break;

            case B:
                if (robot.wrist != null && pressed &&
                    (elbowPos > Elbow.Params.SAFE_POS || extenderPos < Extender.Params.MAX_POS - 6.0))
                {
                    robot.wrist.setPosition(Wrist.Params.HIGH_CHAMBER_SCORE_POS);
                }
                break;

            case X:
                if (robot.wrist != null && pressed)
                {
                    robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS);
                }
                break;

            case Y:
                if (driverAltFunc && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel all.");
                    robot.cancelAll();
                }
                break;

            case LeftBumper:
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + pressed);
                operatorAltFunc = pressed;
                break;

            case RightBumper:
                if (robot.extenderArm != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Turtle.");
                    robot.extenderArm.retract(null);
                }
                break;

            case DpadUp:
                if (robot.wrist != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist preset up.");
                    robot.wrist.presetPositionUp(null);
                }
                break;

            case DpadDown:
                if (robot.wrist != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist preset down.");
                    robot.wrist.presetPositionDown(null);
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
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Manual intake.");
                            robot.grabber.intake(null, 0.0, null);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Auto intake.");
                            robot.grabber.autoIntake(null, 0.0, Grabber.Params.FINISH_DELAY, null);
                        }
                    }
                    else
                    {
                        if (robot.grabber.isAutoActive())
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto intake.");
                            robot.grabber.cancel();
                        }
                        else
                        {
                            // This is manual override in case the sensor is not working, just turn it OFF.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Stop manual intake.");
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
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Manual dump.");
                            robot.grabber.dump(null, 0.0, null);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Auto dump.");
                            robot.grabber.autoDump(null, 0.0, Grabber.Params.DUMP_DELAY, null);
                        }
                    }
                    else
                    {
                        if (robot.grabber.isAutoActive())
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel auto dump.");
                            robot.grabber.cancel();
                        }
                        else
                        {
                            // This is manual override in case the sensor is not working, just turn it OFF.
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Stop manual dump.");
                            robot.grabber.stop(null);
                        }
                    }
                }
                break;

            case Back:
                if (pressed)
                {
                    // Zero calibrate all subsystems (arm, elevator and turret).
                    robot.globalTracer.traceInfo(moduleName, ">>>>> ZeroCalibrating.");
                    robot.cancelAll();
                    robot.zeroCalibrate(moduleName);
                }
                break;

            case Start:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
