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

import teamcode.Robot;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Wrist;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskExtenderArm extends TrcAutoTask<TaskExtenderArm.State>
{
    private static final String moduleName = TaskExtenderArm.class.getSimpleName();

    public enum State
    {
        SET_POSITION,
        RETRACT_EXTENDER,
        SET_ELBOW_ANGLE,
        SET_EXTENDER_POSITION,
        CHECK_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams
    {
        Double elbowAngle;
        Double extenderPosition;
        Double wristPosition;

        TaskParams(Double elbowAngle, Double extenderPosition, Double wristPosition)
        {
            this.elbowAngle = elbowAngle;
            this.extenderPosition = extenderPosition;
            this.wristPosition = wristPosition;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent elbowEvent;
    private final TrcEvent extenderEvent;

    private String currOwner = null;
    private boolean safeSequence = false;
    private boolean elbowCompleted = false;
    private boolean extenderCompleted = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskExtenderArm(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        this.elbowEvent = new TrcEvent(Elbow.Params.SUBSYSTEM_NAME);
        this.extenderEvent = new TrcEvent(Extender.Params.SUBSYSTEM_NAME);
    }   //TaskExtenderArm

    /**
     * This method sets the Elbow, Extender and Wrist to their specifies positions.
     *
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param wristPosition specifies the wrist position, null if not moving wrist.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        boolean safeSequence, Double elbowAngle, Double extenderPosition, Double wristPosition,
        TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName,
            "safeSequence=" + safeSequence +
            "elbowAngle=" + elbowAngle +
            ", extenderPos=" + extenderPosition +
            ", wristPosition=" + wristPosition +
            ", event=" + completionEvent);
        this.safeSequence = safeSequence;
        startAutoTask(
            State.SET_POSITION, new TaskParams(elbowAngle, extenderPosition, wristPosition), completionEvent);
    }   //setPosition

    /**
     * This method sets the Elbow, Extender and Wrist to their specifies positions.
     *
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param wristPosition specifies the wrist position, null if not moving wrist.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(Double elbowAngle, Double extenderPosition, Double wristPosition, TrcEvent completionEvent)
    {
        setPosition(false, elbowAngle, extenderPosition, wristPosition, completionEvent);
    }   //setPosition

    /**
     * This method retracts everything.
     *
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(boolean safeSequence, TrcEvent completionEvent)
    {
        setPosition(
            safeSequence, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, Wrist.Params.MIN_POS, completionEvent);
    }   //retract

    /**
     * This method retracts everything.
     *
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(TrcEvent completionEvent)
    {
        setPosition(false, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, Wrist.Params.GROUND_PICKUP_POS, completionEvent);
    }   //retract

    /**
     * This method cancels the ExtenderArm AutoTask.
     */
    public void cancel()
    {
        tracer.traceInfo(moduleName, "Canceling AutoTask");
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
                          robot.elbow.acquireExclusiveAccess(ownerName) &&
                          robot.extender.acquireExclusiveAccess(ownerName) &&
                          robot.wrist.acquireExclusiveAccess(ownerName);

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
                ", elbow=" + ownershipMgr.getOwner(robot.elbow) +
                ", extender=" + ownershipMgr.getOwner(robot.extender) +
                ", wrist=" + ownershipMgr.getOwner(robot.wrist) + ").");
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
                ", elbow=" + ownershipMgr.getOwner(robot.elbow) +
                ", extender=" + ownershipMgr.getOwner(robot.extender) +
                ", wrist=" + ownershipMgr.getOwner(robot.wrist) + ").");
            robot.elbow.releaseExclusiveAccess(currOwner);
            robot.extender.releaseExclusiveAccess(currOwner);
            robot.wrist.releaseExclusiveAccess(currOwner);
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
        robot.elbow.cancel();
        robot.extender.cancel();
        robot.wrist.cancel();
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
            case SET_POSITION:
                elbowCompleted = false;
                extenderCompleted = false;
                if (taskParams.wristPosition != null)
                {
                    // Caller provided wrist position, go set it (fire and forget).
                    robot.wrist.setPosition(currOwner, 0.0, taskParams.wristPosition, null, 0.0);
                }
                sm.setState(safeSequence? State.RETRACT_EXTENDER: State.SET_ELBOW_ANGLE);
                break;

            case RETRACT_EXTENDER:
                if (taskParams.elbowAngle != null &&
                    Math.abs(robot.extender.getPosition() - Extender.Params.MIN_POS) >
                    Extender.Params.POS_PID_TOLERANCE)
                {
                    // We are setting the elbow angle and the extender is extended, retract it first.
                    robot.extender.setPosition(
                        currOwner, 0.0, Extender.Params.MIN_POS, true, Extender.Params.POWER_LIMIT, extenderEvent, 0.0);
                    sm.waitForSingleEvent(extenderEvent, State.SET_ELBOW_ANGLE);
                }
                else
                {
                    // Either we are not setting elbow angle or the extender is already retracted, skip this state.
                    sm.setState(State.SET_ELBOW_ANGLE);
                }
                break;

            case SET_ELBOW_ANGLE:
                if (taskParams.elbowAngle != null)
                {
                    elbowEvent.setCallback((c)->{elbowCompleted = true;}, null);
                    // We are setting elbow angle, go do it.
                    robot.elbow.setPosition(
                        currOwner, 0.0, taskParams.elbowAngle, true, Elbow.Params.POWER_LIMIT, elbowEvent, 4.0);
                    if (safeSequence)
                    {
                        // Don't need a callback if we are waiting for its completion.
                        elbowEvent.setCallback(null, null);
                        elbowCompleted = true;
                        sm.waitForSingleEvent(elbowEvent, State.SET_EXTENDER_POSITION);
                    }
                    else
                    {
                        sm.setState(State.SET_EXTENDER_POSITION);
                    }
                }
                else
                {
                    // Caller did not provide elbow angle, skip this state.
                    elbowCompleted = true;
                    sm.setState(State.SET_EXTENDER_POSITION);
                }
                break;

            case SET_EXTENDER_POSITION:
                if (taskParams.extenderPosition != null)
                {
                    extenderEvent.setCallback((c)->{extenderCompleted = true;}, null);
                    // We are setting extender position, go do it.
                    robot.extender.setPosition(
                        currOwner, 0.0, taskParams.extenderPosition, true, Extender.Params.POWER_LIMIT, extenderEvent,
                        4.0);
                    if (safeSequence)
                    {
                        // Don't need a callback if we are wait for its completion.
                        extenderEvent.setCallback(null, null);
                        extenderCompleted = true;
                        sm.waitForSingleEvent(extenderEvent, State.CHECK_COMPLETION);
                    }
                    else
                    {
                        sm.setState(State.CHECK_COMPLETION);
                    }
                }
                else
                {
                    // We are not setting extender position, we are done.
                    extenderCompleted = true;
                    sm.setState(State.CHECK_COMPLETION);
                }
                break;

            case CHECK_COMPLETION:
                if (elbowCompleted && extenderCompleted)
                {
                    sm.setState(State.DONE);
                }
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskExtenderArm
