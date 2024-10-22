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
import teamcode.subsystems.Intake;
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
        final FtcAuto.StartPos startPos;
        final boolean doDrive;

        TaskParams(FtcAuto.Alliance alliance, FtcAuto.ScoreHeight scoreHeight, FtcAuto.StartPos startPos, boolean doDrive)
        {
            this.alliance = alliance;
            this.scoreHeight = scoreHeight;
            this.startPos = startPos;
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
     * @param alliance specifies the alliance color.
     * @param scoreHeight specifies the scoring height.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreChamber(FtcAuto.Alliance alliance, FtcAuto.ScoreHeight scoreHeight, FtcAuto.StartPos startPos, boolean doDrive, TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName, "alliance=" + alliance + ",scoreHeight=" + scoreHeight + ",event=" + completionEvent);
        startAutoTask(State.GO_TO_SCORE_POSITION, new TaskParams(alliance, scoreHeight, startPos, doDrive), completionEvent);
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
                if (taskParams.doDrive)
                {
//                    TrcPose2D scorePose = taskParams.alliance == FtcAuto.Alliance.RED_ALLIANCE?
//                            GameParams.RED_CHAMBER_SCORE_POSE: GameParams.BLUE_CHAMBER_SCORE_POSE;
                    TrcPose2D scorePose;
                    if (taskParams.alliance != null)
                    {
                        if (taskParams.alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            if (taskParams.startPos == FtcAuto.StartPos.OBSERVATION_ZONE)
                            {
                                scorePose = GameParams.RED_OBSERVATION_CHAMBER_SCORE_POSE;
                            } else
                            {
                                scorePose = GameParams.RED_BASKET_CHAMBER_SCORE_POSE;
                            }
                        }
                        else
                        {
                            if (taskParams.startPos == FtcAuto.StartPos.OBSERVATION_ZONE)
                            {
                                scorePose = GameParams.BLUE_OBSERVATION_CHAMBER_SCORE_POSE;
                            }
                            else
                            {
                                scorePose = GameParams.BLUE_BASKET_CHAMBER_SCORE_POSE;
                            }
                        }
                    } else
                    {
                        if (robot.robotDrive.driveBase.getFieldPosition().y < 0)
                        // Red Alliance
                        {
                            if (robot.robotDrive.driveBase.getFieldPosition().x >= 0)
                            {
                                scorePose = GameParams.RED_OBSERVATION_CHAMBER_SCORE_POSE;
                            } else
                            {
                                scorePose = GameParams.RED_BASKET_CHAMBER_SCORE_POSE;
                            }
                        } else
                        // Blue Alliance
                        {
                            if (robot.robotDrive.driveBase.getFieldPosition().x >= 0)
                            {
                                scorePose = GameParams.BLUE_OBSERVATION_CHAMBER_SCORE_POSE;
                            } else
                            {
                                scorePose = GameParams.BLUE_BASKET_CHAMBER_SCORE_POSE;
                            }
                        }
                    }
                    robot.robotDrive.purePursuitDrive.start(
                            currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), false, scorePose);
                    sm.waitForSingleEvent(event, State.SET_ELBOW);
                }
                else
                {
                    sm.setState(State.SET_ELBOW);
                }
                break;

            case SET_ELBOW:
                double elbowAngle;
                if (taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW)
                {
                    elbowAngle = GameParams.CHAMBER_LOW_ELBOW_ANGLE;
                }
                else
                {
                    elbowAngle = GameParams.CHAMBER_HIGH_ELBOW_ANGLE;
                }
                robot.extenderArm.setPosition(elbowAngle, null, null, event);
                sm.waitForSingleEvent(event, State.SET_EXTENDER);
                break;

            case SET_EXTENDER:
                double extenderPos;
                if (taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW)
                {
                    extenderPos = GameParams.CHAMBER_LOW_EXTENDER_POS;
                }
                else
                {
                    extenderPos = GameParams.CHAMBER_HIGH_EXTENDER_POS;
                }
                robot.extenderArm.setPosition(null, extenderPos, null, event);
                sm.waitForSingleEvent(event, State.SCORE_CHAMBER);
                break;

            case SCORE_CHAMBER:
                double wristPos = taskParams.scoreHeight == FtcAuto.ScoreHeight.LOW?
                        GameParams.CHAMBER_LOW_WRIST_SCORE_POS: GameParams.CHAMBER_HIGH_WRIST_SCORE_POS;
                robot.wrist.setPosition(wristPos, event, Wrist.Params.DUMP_TIME);
                sm.waitForSingleEvent(event, State.RETRACT_EXTENDER_ARM);
                break;

            case RETRACT_EXTENDER_ARM:
                robot.intake.autoEjectReverse(
                        0.0,Intake.Params.REVERSE_POWER,
                       Intake.Params.FINISH_DELAY, null, 4.0);
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
