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
//        RETRACT_EXTENDER_ARM,
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
        final boolean doDrive;

        TaskParams(
            FtcAuto.Alliance alliance, TrcPose2D scorePose, double elbowAngle, double extenderPos, double wristPos,
            boolean doDrive)
        {
            this.alliance = alliance;
            this.scorePose = scorePose;
            this.elbowAngle = elbowAngle;
            this.extenderPos = extenderPos;
            this.wristPos = wristPos;
            this.doDrive = doDrive;
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
     * @param alliance specifies the alliance color, can be null if caller is TeleOp.
     * @param startPos specifies the autoonomous robot start position, can be null if caller is TeleOp.
     * @param scoreHeight specifies the scoring height.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(
        FtcAuto.Alliance alliance, FtcAuto.StartPos startPos, Robot.ScoreHeight scoreHeight, boolean doDrive,
        TrcEvent completionEvent)
    {
        TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
        TrcPose2D scorePose;
        double elbowAngle, extenderPos, wristPos;

        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            alliance = robotPose.y < 0.0? FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        if (startPos == null)
        {
            // Caller is TeleOp, let's determine the score position by robot's current location.
            scorePose = robotPose.x < 0.0?
                GameParams.RED_NET_CHAMBER_SCORE_POSE: GameParams.RED_OBSERVATION_CHAMBER_SCORE_POSE;
        }
        else
        {
            // Caller is Auto, let's determine the score position by robot's start position.
            scorePose = startPos == FtcAuto.StartPos.NET_ZONE?
                GameParams.RED_NET_CHAMBER_SCORE_POSE: GameParams.RED_OBSERVATION_CHAMBER_SCORE_POSE;
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

        tracer.traceInfo(
            moduleName,
            "alliance=" + alliance +
            ",scorePose=" + scorePose +
            ",elbowAngle=" + elbowAngle +
            ",extenderPos=" + extenderPos +
            ",wristPos=" + wristPos +
            ",doDrive=" + doDrive +
            ",event=" + completionEvent);
        startAutoTask(
            State.GO_TO_SCORE_POSITION,
            new TaskParams(alliance, scorePose, elbowAngle, extenderPos, wristPos, doDrive), completionEvent);
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
            case GO_TO_SCORE_POSITION:
                // Code Review: Is it really realistic to not drive??? If you don't drive, who guarantee the robot
                // is at the correct scoring location?
                if (taskParams.doDrive)
                {
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.adjustPoseByAlliance(taskParams.scorePose, taskParams.alliance, false));
                    robot.extenderArm.setPosition(taskParams.elbowAngle, null, taskParams.wristPos, null);
                    sm.waitForSingleEvent(event, State.SET_EXTENDER);
                }
                else
                {
                    robot.extenderArm.setPosition(taskParams.elbowAngle, null, null, null);
                    sm.setState(State.SET_EXTENDER);
                }
                break;

//            case SET_ELBOW:
//                robot.extenderArm.setPosition(taskParams.elbowAngle, null, null, event);
//                sm.waitForSingleEvent(event, State.SET_EXTENDER);
//                break;

            case SET_EXTENDER:
                robot.extenderArm.setPosition(null, taskParams.extenderPos, null, event);
                sm.waitForSingleEvent(event, State.LOWER_ELBOW);
                break;

            case LOWER_ELBOW:
                robot.extenderArm.setPosition(taskParams.elbowAngle - 20.0, null, null, event);
//                robot.extenderArm.setPosition(null, null, taskParams.wristPos, null);
                sm.waitForSingleEvent(event, State.SCORE_CHAMBER);
                break;

            case SCORE_CHAMBER:
                robot.grabber.autoDump(currOwner, 0.0, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

//            case RETRACT_EXTENDER_ARM:
//                robot.extenderArm.retract(event);
//                sm.waitForSingleEvent(event, State.DONE);
//                break;
//            case DRIVE_BACK:
//                robot.robotDrive.purePursuitDrive.start(
//                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
//                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
//                        ;
//                sm.waitForSingleEvent(event, State.DONE);

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoScoreChamber
