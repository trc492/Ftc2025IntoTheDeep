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

import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import trclib.motor.TrcMotor;
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
        WAIT_FOR_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams
    {
        Double elbowAngle;
        Double extenderPosition;

        TaskParams(Double elbowAngle, Double extenderPosition)
        {
            this.elbowAngle = elbowAngle;
            this.extenderPosition = extenderPosition;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "elbowPos=" + elbowAngle + ",extenderPos=" + extenderPosition ;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    public final TrcMotor elbow;
    public final TrcMotor extender;
    private final TrcEvent elbowEvent;
    private final TrcEvent extenderEvent;

    private String currOwner = null;
    private boolean safeSequence = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param elbow specifies the elbow object.
     * @param extender specifies the extender object.
     */
    public TaskExtenderArm(String ownerName, TrcMotor elbow, TrcMotor extender)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.elbow = elbow;
        this.extender = extender;
        this.elbowEvent = new TrcEvent(Elbow.Params.SUBSYSTEM_NAME);
        this.extenderEvent = new TrcEvent(Extender.Params.SUBSYSTEM_NAME);
    }   //TaskExtenderArm

    /**
     * This method zero calibrates the ExtenderArm. This includes zero calibrating both the elbow and the extender.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the completion event to signal if provided.
     */
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Strictly speaking, this is not an autotask operation because it is not calling startAutoTask.
        // Therefore, it does not acquire subsystem ownership for the caller. It's the responsibility of the caller
        // to acquire ownership if desired, or pass in null owner if no ownership required.
        tracer.traceInfo(moduleName, "Zero Calibrating.");
        if (completionEvent != null)
        {
            elbowEvent.setCallback(this::zeroCalibrateCallback, completionEvent);
            elbow.zeroCalibrate(owner, Elbow.Params.ZERO_CAL_POWER, elbowEvent);
            extenderEvent.setCallback(this::zeroCalibrateCallback, completionEvent);
            extender.zeroCalibrate(owner, Extender.Params.ZERO_CAL_POWER, extenderEvent);
        }
        else
        {
            elbow.zeroCalibrate(owner, Elbow.Params.ZERO_CAL_POWER);
            extender.zeroCalibrate(owner, Extender.Params.ZERO_CAL_POWER);
        }
    }   //zeroCalibrate

    /**
     * This method is called when either elbow or extender zero calibration is done.
     *
     * @param context specifies the completion event to signal when both elbow and extender zero calibration is done.
     */
    private void zeroCalibrateCallback(Object context)
    {
        if (elbowEvent.isSignaled() && extenderEvent.isSignaled())
        {
            ((TrcEvent) context).signal();
        }
    }   //zeroCalibrateCallback

    /**
     * This method sets the Elbow and Extender to their specifies positions.
     *
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        boolean safeSequence, Double elbowAngle, Double extenderPosition, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(elbowAngle, extenderPosition);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        this.safeSequence = safeSequence;
        startAutoTask(State.SET_POSITION, taskParams, completionEvent);
    }   //setPosition

    /**
     * This method sets the Elbow and Extender to their specifies positions.
     *
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(Double elbowAngle, Double extenderPosition, TrcEvent completionEvent)
    {
        setPosition(false, elbowAngle, extenderPosition, completionEvent);
    }   //setPosition

    /**
     * This method retracts everything.
     *
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(boolean safeSequence, TrcEvent completionEvent)
    {
        setPosition(safeSequence, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, completionEvent);
    }   //retract

    /**
     * This method retracts everything.
     *
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(TrcEvent completionEvent)
    {
        setPosition(false, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, completionEvent);
    }   //retract

    /**
     * This method cancels the ExtenderArm AutoTask.
     */
    public void cancel()
    {
        tracer.traceInfo(moduleName, "Canceling AutoTask");
        stopAutoTask(false);
        // stopAutoTask only stop subsystems if auto task is active.
        // If subsystems are active not as part of AutoTask operation (e.g. zeroCalibrate), stopAutoTask won't do
        // anything. Let's cancel the subsystems explicitly.
        stopSubsystems();
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
                          elbow.acquireExclusiveAccess(ownerName) &&
                          extender.acquireExclusiveAccess(ownerName);

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
                ", elbow=" + ownershipMgr.getOwner(elbow) +
                ", extender=" + ownershipMgr.getOwner(extender) + ").");
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
                ", elbow=" + ownershipMgr.getOwner(elbow) +
                ", extender=" + ownershipMgr.getOwner(extender) + ").");
            elbow.releaseExclusiveAccess(currOwner);
            extender.releaseExclusiveAccess(currOwner);
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
        elbow.cancel();
        extender.cancel();
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
                elbowEvent.clear();
                extenderEvent.clear();
                sm.setState(safeSequence? State.RETRACT_EXTENDER: State.SET_ELBOW_ANGLE);
                break;

            case RETRACT_EXTENDER:
                if (taskParams.elbowAngle != null &&
                    Math.abs(extender.getPosition() - Extender.Params.MIN_POS) >
                    Extender.Params.POS_PID_TOLERANCE)
                {
                    // We are setting the elbow angle and the extender is extended, retract it first.
                    extender.setPosition(
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
                    // We are setting elbow angle, go do it.
                    elbow.setPosition(
                        currOwner, 0.0, taskParams.elbowAngle, true, Elbow.Params.POWER_LIMIT, elbowEvent, 4.0);
                    if (safeSequence)
                    {
                        sm.waitForSingleEvent(elbowEvent, State.SET_EXTENDER_POSITION);
                    }
                    else
                    {
                        // Not performing safe sequence, so don't wait.
                        sm.setState(State.SET_EXTENDER_POSITION);
                    }
                }
                else
                {
                    // Caller did not provide elbow angle, skip this state.
                    elbowEvent.signal();
                    sm.setState(State.SET_EXTENDER_POSITION);
                }
                break;

            case SET_EXTENDER_POSITION:
                if (taskParams.extenderPosition != null)
                {
                    // We are setting extender position, go do it.
                    extender.setPosition(
                        currOwner, 0.0, taskParams.extenderPosition, true, Extender.Params.POWER_LIMIT, extenderEvent,
                        4.0);
                    if (safeSequence)
                    {
                        sm.waitForSingleEvent(extenderEvent, State.WAIT_FOR_COMPLETION);
                    }
                    else
                    {
                        // Not performing safe sequence, so don't wait.
                        sm.setState(State.WAIT_FOR_COMPLETION);
                    }
                }
                else
                {
                    // We are not setting extender position, we are done.
                    extenderEvent.signal();
                    sm.setState(State.WAIT_FOR_COMPLETION);
                }
                break;

            case WAIT_FOR_COMPLETION:
                if (safeSequence)
                {
                    // If we performed safe sequence and came here, it means both events are already signaled.
                    sm.setState(State.DONE);
                }
                else
                {
                    sm.addEvent(elbowEvent);
                    sm.addEvent(extenderEvent);
                    // Don't clear the events.
                    sm.waitForEvents(State.DONE, false, true);
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
