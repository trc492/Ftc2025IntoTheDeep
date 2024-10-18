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
import teamcode.RobotParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist scoring specimen on the chamber.
 */
public class TaskAutoScoreChamber extends TrcAutoTask<TaskAutoScoreChamber.State>
{
    private static final String moduleName = TaskAutoScoreChamber.class.getSimpleName();

    public enum State
    {
        GO_TO_SCORE_POSITION,
        SET_ELBOW,
        SET_EXTENDER,
        SCORE_CHAMBER,
        RETRACT_EXTENDER_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final FtcAuto.ScoreHeight scoreHeight;

        TaskParams(FtcAuto.Alliance alliance, FtcAuto.ScoreHeight scoreHeight)
        {
            this.alliance = alliance;
            this.scoreHeight = scoreHeight;
        }  //TaskParams
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
    public TaskAutoScoreChamber(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.event = new TrcEvent(moduleName);
    }   //TaskAutoScoreChamber

    /**
     * This method starts the auto-assist operation.
     *
     * @param alliance specifies the alliance color.
     * @param scoreHeight specifies the scoring height.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(FtcAuto.Alliance alliance, FtcAuto.ScoreHeight scoreHeight, TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName, "alliance=" + alliance + ",scoreHeight=" + scoreHeight + ",event=" + completionEvent);
        startAutoTask(State.GO_TO_SCORE_POSITION, new TaskParams(alliance, scoreHeight), completionEvent);
    }   //autoScoreChamber

    /**
     * This method cancels an in progress auto-assist operation if any.
     */
    public void cancel()
    {
        tracer.traceInfo(moduleName, "Canceling operation.");
        stopAutoTask(false);
    }   //cancel

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
        boolean success = ownerName == null ||
                          (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));

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
            case GO_TO_SCORE_POSITION:
                TrcPose2D scorePose = taskParams.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                    RobotParams.Game.RED_CHAMBER_SCORE_POSE: RobotParams.Game.BLUE_CHAMBER_SCORE_POSE;
                robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false, scorePose);
                sm.waitForSingleEvent(event, State.SET_ELBOW);
                break;

            case SET_ELBOW:
                double elbowAngle;
                if (taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW)
                {
                    elbowAngle = RobotParams.Game.CHAMBER_LOW_ELBOW_ANGLE;
                }
                else
                {
                    elbowAngle = RobotParams.Game.CHAMBER_HIGH_ELBOW_ANGLE;
                }
                robot.extenderArm.setPosition(elbowAngle, null, null, event);
                sm.waitForSingleEvent(event, State.SET_EXTENDER);
                break;

            case SET_EXTENDER:
                double extenderPos;
                if (taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW)
                {
                    extenderPos = RobotParams.Game.CHAMBER_LOW_EXTENDER_POS;
                }
                else
                {
                    extenderPos = RobotParams.Game.CHAMBER_HIGH_EXTENDER_POS;
                }
                robot.extenderArm.setPosition(null, extenderPos, null, event);
                sm.waitForSingleEvent(event, State.SCORE_CHAMBER);
                break;

            case SCORE_CHAMBER:
                double wristPos = taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW?
                        RobotParams.Game.CHAMBER_LOW_WRIST_SCORE_POS: RobotParams.Game.CHAMBER_HIGH_WRIST_SCORE_POS;
                robot.wrist.setPosition(wristPos, event, RobotParams.WristParams.DUMP_TIME);
                sm.waitForSingleEvent(event, State.RETRACT_EXTENDER_ARM);
                break;

            case RETRACT_EXTENDER_ARM:
                robot.extenderArm.retract(event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoScoreChamber
