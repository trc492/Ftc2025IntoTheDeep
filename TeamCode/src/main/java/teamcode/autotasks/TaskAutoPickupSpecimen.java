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

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.params.GameParams;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Wrist;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

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
        PICKUP_SPECIMEN,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        TaskParams(FtcAuto.Alliance alliance)
        {
            this.alliance = alliance;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;

    private String currOwner = null;

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
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupSpecimen(FtcAuto.Alliance alliance, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        tracer.traceInfo(moduleName, "alliance=" + alliance + ",event=" + completionEvent);
        startAutoTask(State.START, new TaskParams(alliance), completionEvent);
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
                          robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName) &&
                          robot.grabber.acquireExclusiveAccess(ownerName);

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
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) +
                ", grabber=" + robot.grabber.getOwner() + ").");
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
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) +
                ", grabber=" + robot.grabber.getOwner() + ").");
            robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
            robot.grabber.releaseExclusiveAccess(currOwner);
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
                if (robot.extenderArm != null)
                {
                    robot.extenderArm.setPosition(
                        Elbow.Params.SPECIMEN_PICKUP_POS, Extender.Params.SPECIMEN_PICKUP_POS,
                        Wrist.Params.HIGH_CHAMBER_SCORE_POS, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_PICKUP);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case DRIVE_TO_PICKUP:
                // Code Review: Can you really do a blink pick up???
                // Drive to the observation zone
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                    robot.adjustPoseByAlliance(GameParams.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                break;

            case PICKUP_SPECIMEN:
                // intake design not confirmed
                // TODO: There will be a color sensor on the intake.
                robot.grabber.autoIntake(currOwner, 0.0, event);
                // based on tuning
                robot.robotDrive.driveBase.holonomicDrive(currOwner, 0.0, 0.15, 0.0);
                sm.waitForSingleEvent(event, State.DONE, 4.0);
                break;

            default:
            case DONE:
                // Stop task.
                if (robot.extenderArm != null)
                {
                    robot.extenderArm.setPosition(
                        Elbow.Params.SPECIMEN_PICKUP_POS + 10.0,
                        Extender.Params.MIN_POS,
                        Wrist.Params.HIGH_CHAMBER_SCORE_POS,
                        null);
                }

                if (robot.grabber != null && robot.ledIndicator != null)
                {
                    robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), false);
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupSpecimen
