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
import teamcode.subsystems.Wrist;
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
//        SET_ELBOW,
        SET_EXTENDER,
        LOWER_ELBOW,
        SCORE_CHAMBER,
        RETRACT_EXTENDER_ARM,
        DRIVE_BACK,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final TrcPose2D scorePose;
        final double elbowAngle;
        final double extenderPos;
        final double wristPos;

        TaskParams(
            FtcAuto.Alliance alliance, TrcPose2D scorePose, double elbowAngle, double extenderPos, double wristPos)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.elbowAngle = elbowAngle;
            this.extenderPos = extenderPos;
            this.wristPos = wristPos;
        }  //TaskParams

        @NonNull
        public String toString()
        {
            return String.format(
                Locale.US, "alliance=%s,scorePose=%s,elbowPos=%.1f,extenderPos=%.1f,wristPos=%.3f",
                alliance, scorePose, elbowAngle, extenderPos, wristPos);
        }   //toString
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
     * @param scoreHeight specifies the scoring height.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(Robot.ScoreHeight scoreHeight, TrcEvent completionEvent)
    {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        FtcAuto.Alliance alliance = robotPose.y < 0.0? FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        boolean nearNetZone = alliance == FtcAuto.Alliance.RED_ALLIANCE ^ robotPose.x > 0.0;
        TrcPose2D scorePose = nearNetZone?
            RobotParams.Game.RED_NET_CHAMBER_SCORE_POSE: RobotParams.Game.RED_OBSERVATION_CHAMBER_SCORE_POSE;
        double elbowAngle, extenderPos, wristPos;

        if (robotPose.x >= -RobotParams.Game.CHAMBER_MAX_SCORE_POS_X &&
            robotPose.x <= RobotParams.Game.CHAMBER_MAX_SCORE_POS_X)
        {
            // If robot current position is within the chamber zone, use its X position.
            scorePose.x = robotPose.x;
        }

        if (scoreHeight == Robot.ScoreHeight.LOW)
        {
            elbowAngle = Elbow.Params.LOW_CHAMBER_SCORE_POS;
            extenderPos = Extender.Params.LOW_CHAMBER_SCORE_POS;
            wristPos = Wrist.Params.LOW_CHAMBER_SCORE_POS;
        }
        else
        {
            elbowAngle = Elbow.Params.HIGH_CHAMBER_SCORE_POS;
            extenderPos = Extender.Params.HIGH_CHAMBER_SCORE_POS;
            wristPos = Wrist.Params.HIGH_CHAMBER_SCORE_POS;
        }

        TaskParams taskParams = new TaskParams(alliance, scorePose, elbowAngle, extenderPos, wristPos);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.GO_TO_SCORE_POSITION, taskParams, completionEvent);
    }   //autoScoreChamber

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
            case GO_TO_SCORE_POSITION:
                robot.robotDrive.purePursuitDrive.start(
                    currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                    robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                    robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance));
                robot.extenderArm.setPosition(taskParams.elbowAngle, null, taskParams.wristPos, null);
                sm.waitForSingleEvent(event, State.SET_EXTENDER);
                break;

            case SET_EXTENDER:
                robot.extenderArm.setPosition(null, taskParams.extenderPos, null, event);
                sm.waitForSingleEvent(event, State.LOWER_ELBOW);
                break;

            case LOWER_ELBOW:
                robot.extenderArm.setPosition(taskParams.elbowAngle - 25.0, null, null, event);
                sm.waitForSingleEvent(event, State.SCORE_CHAMBER);
                break;

            case SCORE_CHAMBER:
                robot.grabber.autoDump(null, 0.0, event);
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
