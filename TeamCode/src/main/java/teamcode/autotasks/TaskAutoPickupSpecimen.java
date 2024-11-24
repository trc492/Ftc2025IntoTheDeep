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

package teamcode.autotasks;

import androidx.annotation.NonNull;

import java.util.Locale;

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Wrist;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements auto-assist task to pick up a specimen from ground.
 */
public class TaskAutoPickupSpecimen extends TrcAutoTask<TaskAutoPickupSpecimen.State>
{
    private static final String moduleName = TaskAutoPickupSpecimen.class.getSimpleName();

    public enum State
    {
        START,
        DRIVE_TO_PICKUP,
        APPROACH_SPECIMEN,
        FIND_SPECIMEN,
        ALIGN_TO_SPECIMEN,
        PICKUP_SPECIMEN,
        RETRACT_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final boolean useVision;
        TaskParams(FtcAuto.Alliance alliance, boolean useVision)
        {
            this.alliance = alliance;
            this.useVision = useVision;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "alliance=" + alliance + ",useVision=" + useVision;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;

    private String currOwner = null;
    private TrcPose2D specimenPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupSpecimen(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
    }   //TaskAutoPickupSpecimen

    /**
     * This method starts the auto-assist operation.
     *
     * @param alliance specifies the alliance color, can be null if caller is TeleOp.
     * @param useVision specifies true to use Vision, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupSpecimen(FtcAuto.Alliance alliance, boolean useVision, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance, useVision);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.START, taskParams, completionEvent);
    }   //autoPickupSpecimen

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership()
    {
        // ExtenderArm is an AutoTask and is not an ExclusiveSubsystem.
        boolean success = ownerName == null ||
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName);

        if (success)
        {
            currOwner = ownerName;
            tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        }
        else
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceWarn(
                moduleName,
                "Failed to acquire subsystem ownership (currOwner=" + currOwner +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            releaseSubsystemsOwnership();
        }

        return success;
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     */
    @Override
    protected void releaseSubsystemsOwnership()
    {
        if (ownerName != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership (currOwner=" + currOwner +
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     */
    @Override
    protected void stopSubsystems()
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.robotDrive.cancel(currOwner);
        robot.grabber.cancel();
        robot.extenderArm.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                // Prep subsystems for the pickup.
                if (robot.extenderArm == null || robot.grabber == null)
                {
                    // Arm or grabber don't exist, nothing we can do.
                    tracer.traceInfo(moduleName, "Arm or grabber doesn't exist, we are done.");
                    sm.setState(State.DONE);
                }
                else
                {
                    // Fire and forget to save time.
                    robot.wrist.setPosition(Wrist.Params.SPECIMEN_PICKUP_POS, null);
                    robot.extenderArm.setPosition(
                        Elbow.Params.SPECIMEN_PICKUP_POS, Extender.Params.SPECIMEN_PICKUP_POS, null);
                    sm.setState(State.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:
                // Drive to the specimen pickup location.
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                    robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                sm.waitForSingleEvent(event, taskParams.useVision? State.FIND_SPECIMEN: State.APPROACH_SPECIMEN);
                break;

            case FIND_SPECIMEN:
                // Use vision to find the sample on the wall.
                specimenPose = robot.getDetectedSamplePose(
                    Robot.sampleType, RobotParams.Game.SPECIMEN_GROUND_OFFSET, true);
                if (specimenPose != null)
                {
                    // Vision found the specimen.
                    String msg = String.format(
                        Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f",
                        Robot.sampleType, specimenPose.x, specimenPose.y, specimenPose.angle);
                    tracer.traceInfo(moduleName, msg);
                    robot.speak(msg);
                    sm.setState(State.ALIGN_TO_SPECIMEN);
                }
                else if (visionExpiredTime == null)
                {
                    // Vision doesn't find the specimen, set a 1-second timeout and keep trying.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out and vision still not finding specimen, giving up.
                    tracer.traceInfo(moduleName, "%s not found, we are done.", Robot.sampleType);
                    sm.setState(State.DONE);
                }
                break;

            case ALIGN_TO_SPECIMEN:
                // Vision found the specimen, align the robot to it.
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                double targetHeading = taskParams.alliance == FtcAuto.Alliance.RED_ALLIANCE? 180.0: 0.0;
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, null, 0.0, robotPose, true,
                    robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                    new TrcPose2D(specimenPose.x, 0.0, targetHeading - robotPose.angle));
                sm.waitForSingleEvent(event, State.APPROACH_SPECIMEN);
                break;

            case APPROACH_SPECIMEN:
                // Turn on intake and approach specimen slowly.
                robot.grabber.autoIntake(null, 0.0, Grabber.Params.FINISH_DELAY, event);
                robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.15, 0.0);
                sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN, 2.0);
                break;

            case PICKUP_SPECIMEN:
                // Grabber got the specimen, stop the drive and raise the arm to pick it up.
                robot.robotDrive.driveBase.stop(currOwner);
                robot.grabber.cancel();
                robot.extenderArm.setPosition(Elbow.Params.SPECIMEN_PICKUP_POS + 10.0, null, event);
                sm.waitForSingleEvent(event, State.RETRACT_ARM);
                break;

            case RETRACT_ARM:
                // Retract the arm with "fire and forget".
                robot.extenderArm.retract(null);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                if (robot.grabber != null && robot.ledIndicator != null)
                {
                    // Flash the LED to show whether we got the specimen and what type.
                    robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), true);
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupSpecimen
