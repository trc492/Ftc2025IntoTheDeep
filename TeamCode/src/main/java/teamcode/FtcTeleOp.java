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
import teamcode.subsystems.RumbleIndicator;
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
    private boolean statusUpdateOn = false;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
    private Integer savedLimelightPipeline = null;
    private double elbowPrevPower = 0.0;
    private double extenderPrevPower = 0.0;
    private Robot.ScoreHeight scoreHeight = Robot.ScoreHeight.HIGH;
    private int climbedLevel = 0;

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

        if (RobotParams.Preferences.useRumble)
        {
            robot.driverRumble = new RumbleIndicator("DriverRumble", driverGamepad);
            robot.operatorRumble = new RumbleIndicator("OperatorRumble", operatorGamepad);
        }

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
            if (robot.vision.aprilTagVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling WebCam AprilTagVision.");
                robot.vision.setAprilTagVisionEnabled(true);
            }
            else if (robot.vision.limelightVision != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling Limelight AprilTagVision.");
                robot.vision.setLimelightVisionEnabled(0, true);
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
        robot.globalTracer.traceInfo(
            moduleName, "***** Stopping TeleOp: " + TrcTimer.getCurrentTimeString() + " *****");
        printPerformanceMetrics();

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
            int lineNum = 1;
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

                    if (RobotParams.Preferences.doStatusUpdate || statusUpdateOn)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotDrive: Power=(%.2f,y=%.2f,rot=%.2f),Mode:%s",
                            inputs[0], inputs[1], inputs[2], robot.robotDrive.driveBase.getDriveOrientation());
                    }
                }

                if (elapsedTime > RobotParams.Game.LEVEL1_ASCENT_DEADLINE)
                {
                    if (robot.driverRumble != null)
                    {
                        robot.driverRumble.setRumblePattern(RumbleIndicator.ASCENT_DEADLINE);
                    }

                    if (robot.operatorRumble != null)
                    {
                        robot.operatorRumble.setRumblePattern(RumbleIndicator.ASCENT_DEADLINE);
                    }
                }
            }
            //
            // Other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                double extenderLimit = Extender.Params.MAX_POS;
                // Analog control of subsystems.
//                if (operatorAltFunc && robot.wrist != null && robot.wrist.differentialWrist != null)
//                {
//                    double tiltPower = operatorGamepad.getRightStickY(true);
//                    double rotatePower = operatorGamepad.getRightStickX(true);
//                    robot.wrist.differentialWrist.setPower(tiltPower, rotatePower);
//                }
//                else if (!operatorAltFunc && robot.elbow != null)
//                {
                double elbowPower = operatorGamepad.getRightStickY(true) * Elbow.Params.POWER_LIMIT;
                double elbowPos = robot.elbow.getPosition();
                // Only do this if there is an extender and elbow angle is below the restricted position threshold.
                if (robot.extender != null && elbowPos < Elbow.Params.RESTRICTED_POS_THRESHOLD)
                {
                    /*
                     When operating the elbow up and down, the extender is smart enough to make sure it is
                     within the 42-inch expansion rule. The following math is basically calculating the
                     extender length limit at the current elbow angle:
                     eo = elbowPivotOffset
                     y = extenderLen
                     il = intakeLen
                     theta = elbowAngle
                     xl = y + il
                     xd = distance of (extender + intake) from pivot
                     xpl = extender projected length on the floor (horizontal length limit)
                     alpha = atan(eo / xl)
                     beta = theta - alpha
                     sin(alpha) = eo / xd
                     xd = eo / sin(alpha)
                     xpl = xd * cos(beta)
                     xpl = eo * cos(theta - alpha) / sin(alpha)
                     Note: cos(a - b) = cos(a)*cos(b) + sin(a)*sin(b)
                     xpl = eo * (cos(theta) * cos(alpha) + sin(theta) * sin(alpha)) / sin(alpha)
                     xpl = eo * (cos(theta) * cos(alpha) / sin(alpha) + sin(theta))
                     xpl = eo * (cos(theta) / tan(alpha) + sin(theta))
                     xpl = eo * (cos(theta) / tan(atan(eo / xl)) + sin(theta))
                     xpl = eo * (cos(theta) / (eo / xl) + sin(theta))
                     xpl = eo * (cos(theta) * xl / eo + sin(theta))
                     xpl / eo = cos(theta) * xl / eo + sin(theta)
                     xpl = cos(theta) * xl + sin(theta) * eo
                     xl = (xpl - sin(theta) * eo)) / cos(theta)
                     y + il = (xpl - sin(theta) * eo) / cos(theta)
                     y = (xpl - sin(theta) * eo) / cos(theta) - il
                     */
                    double elbowPosRadians = Math.toRadians(elbowPos);
                    // Assuming grabber MIN_POS is 0-degree, MAX_POS is 180-degree.
                    double grabberAngleRadians = Math.toRadians(robot.wrist.getTiltPosition());
                    double grabberLength = Grabber.Params.GRABBER_LENGTH * Math.cos(grabberAngleRadians);
                    extenderLimit =
                        (Extender.Params.HORIZONTAL_LIMIT - Elbow.Params.PIVOT_OFFSET * Math.sin(elbowPosRadians)) /
                        Math.cos(elbowPosRadians) - grabberLength;
//                    robot.globalTracer.traceInfo(moduleName, "***** Extender Limit: %f", extenderLimit);
                    if (robot.extender.getPosition() > extenderLimit)
                    {
                        robot.extender.setPosition(extenderLimit);
                    }
                }

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
//                }

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
                            robot.extender.setPidPower(extenderPower, Extender.Params.MIN_POS, extenderLimit, true);
                        }
                        extenderPrevPower = extenderPower;
                    }
                }
            }
            // Display subsystem status.
            if (RobotParams.Preferences.doStatusUpdate || statusUpdateOn)
            {
                robot.updateStatus(lineNum);
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
                        robot.scoreChamberTask.autoScoreChamber(scoreHeight, driverAltFunc, null);
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
                        // Code Review: what should we set the "fromSubmersible" to? Do the driver has the say?
                        robot.scoreBasketTask.autoScoreBasket(null, scoreHeight, !driverAltFunc, false, null);
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
                if (driverAltFunc)
                {
                    if (!RobotParams.Preferences.doStatusUpdate)
                    {
                        // Toggle status update ON/OFF.
                        statusUpdateOn = !statusUpdateOn;
                    }
                }
                else
                {
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
                            robot.ledIndicator.setDetectedPattern(LEDIndicator.SCORE_HEIGHT_LOW);
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set score height high.");
                        scoreHeight = Robot.ScoreHeight.HIGH;
                        if (robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setDetectedPattern(LEDIndicator.SCORE_HEIGHT_HIGH);
                        }
                    }
                }
                break;

            case DpadDown:
                if (pressed && robot.autoClimbTask != null)
                {
                    if (climbedLevel == 0)
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto climb level 1.");
                        robot.autoClimbTask.autoClimbLevel1(null);
                        climbedLevel++;
                    }
                    else if (climbedLevel == 1)
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto climb level 2.");
                        robot.autoClimbTask.autoClimbLevel2(null);
                        climbedLevel++;
                    }
                    else if (climbedLevel == 2)
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto climb level 3.");
                        robot.autoClimbTask.autoClimbLevel3(null);
                        climbedLevel++;
                    }
                }
                break;

            case DpadLeft:
                if (robot.pickupFromGroundTask != null && pressed)
                {
                    if (!robot.pickupFromGroundTask.isActive())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto pickup from ground.");
                        robot.pickupFromGroundTask.autoPickupFromGround(Robot.sampleType, true, false, null, null);
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
                        robot.pickupSpecimenTask.autoPickupSpecimen(null, !driverAltFunc, false, null);
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
                    robot.zeroCalibrate(moduleName, null);
                }
                break;

            case Start:
                if (!driverAltFunc)
                {
                    if (robot.vision != null && robot.robotDrive != null)
                    {
                        boolean hasAprilTagVision = robot.vision.isAprilTagVisionEnabled();

                        if (!hasAprilTagVision && robot.vision.limelightVision != null)
                        {
                            hasAprilTagVision = true;
                            if (pressed)
                            {
                                // Webcam AprilTag vision is not enable, enable Limelight AprilTag pipeline instead.
                                savedLimelightPipeline = robot.vision.limelightVision.getPipeline();
                                robot.vision.setLimelightVisionEnabled(0, true);
                            }
                        }

                        if (hasAprilTagVision)
                        {
                            // On press of the button, we will start looking for AprilTag for re-localization.
                            // On release of the button, we will set the robot's field location if we found the
                            // AprilTag.
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
                                    if (savedLimelightPipeline != null)
                                    {
                                        // Done with AprilTag re-localization, restore previous Limelight pipeline.
                                        robot.vision.limelightVision.setPipeline(savedLimelightPipeline);
                                        savedLimelightPipeline = null;
                                    }
                                }
                            }
                            else
                            {
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                            }
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
                if (robot.wrist != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Set wrist to high basket scoring position.");
                    robot.wrist.setPosition(Wrist.Params.LOW_BASKET_SCORE_POS, 0.0);
                }
                break;

            case B:
                if (robot.wrist != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Set wrist to high chamber scoring position.");
                    robot.wrist.setPosition(Wrist.Params.LOW_CHAMBER_SCORE_POS, 0.0);
                }
                break;

            case X:
                if (robot.wrist != null && pressed)
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Set wrist to ground pickup position.");
                    // Don't change rotate position in case the driver has aligned the wrist already.
                    robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS, null);
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
                    if (operatorAltFunc)
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist tilt preset up.");
                        robot.wrist.tiltPresetPositionUp(null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist rotate preset right.");
                        robot.wrist.rotatePresetPositionUp(null);
                    }
                }
                break;

            case DpadDown:
                if (robot.wrist != null && pressed)
                {
                    if (operatorAltFunc)
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist tilt preset down.");
                        robot.wrist.tiltPresetPositionDown(null);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Wrist tilt preset down.");
                        robot.wrist.rotatePresetPositionDown(null);
                    }
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
                    robot.zeroCalibrate(moduleName, null);
                }
                break;

            case Start:
                break;
        }
    }   //operatorButtonEvent

}   //class FtcTeleOp
