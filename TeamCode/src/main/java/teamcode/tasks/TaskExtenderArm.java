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

package teamcode.tasks;

import androidx.annotation.NonNull;

import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements auto-assist task.
 */
public class TaskExtenderArm extends TrcAutoTask<TaskExtenderArm.State>
{
    private static final String moduleName = TaskExtenderArm.class.getSimpleName();

    public enum State
    {
        DO_DELAY,
        SET_POSITION,
        RETRACT_EXTENDER,
        SET_ELBOW_ANGLE,
        SET_EXTENDER_POSITION,
        WAIT_FOR_COMPLETION,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean safeSequence;
        double delay;
        Double elbowAngle;
        Double extenderPosition;

        TaskParams(boolean safeSequence, double delay, Double elbowAngle, Double extenderPosition)
        {
            this.safeSequence = safeSequence;
            this.delay = delay;
            this.elbowAngle = elbowAngle;
            this.extenderPosition = extenderPosition;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "safeSequence=" + safeSequence +
                   ",delay=" + delay +
                   ",elbowPos=" + elbowAngle +
                   ",extenderPos=" + extenderPosition ;
        }   //toString
    }   //class TaskParams

    public final TrcMotor elbow;
    public final TrcMotor extender;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent elbowEvent;
    private final TrcEvent extenderEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param elbow specifies the elbow object.
     * @param extender specifies the extender object.
     */
    public TaskExtenderArm(TrcMotor elbow, TrcMotor extender)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.elbow = elbow;
        this.extender = extender;
        this.timer = new TrcTimer(moduleName);
        this.event = new TrcEvent(moduleName);
        this.elbowEvent = new TrcEvent(Elbow.Params.SUBSYSTEM_NAME);
        this.extenderEvent = new TrcEvent(Extender.Params.SUBSYSTEM_NAME);
    }   //TaskExtenderArm

    /**
     * This method zero calibrates the ExtenderArm. This includes zero calibrating both the elbow and the extender.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
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
     * This method zero calibrates the ExtenderArm. This includes zero calibrating both the elbow and the extender.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the completion event to signal if provided.
     */
    public void stagedZeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Strictly speaking, this is not an autotask operation because it is not calling startAutoTask.
        // Therefore, it does not acquire subsystem ownership for the caller. It's the responsibility of the caller
        // to acquire ownership if desired, or pass in null owner if no ownership required.
        tracer.traceInfo(moduleName, "Staged Zero Calibrating.");
        if (completionEvent != null)
        {
            extenderEvent.setCallback(this::stagedCalibrationCallBack, completionEvent);
            extender.zeroCalibrate(owner, Extender.Params.ZERO_CAL_POWER, extenderEvent);
        }
    }   //zeroCalibrate

    private void stagedCalibrationCallBack(Object context)
    {
        if (extenderEvent.isSignaled())
        {
            elbowEvent.setCallback(this::zeroCalibrateCallback, context);
            elbow.zeroCalibrate(null, Elbow.Params.ZERO_CAL_POWER, elbowEvent);
        }
    }

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
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param delay specifies the delay in seconds for starting the operation.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        String owner, boolean safeSequence, double delay, Double elbowAngle, Double extenderPosition,
        TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(safeSequence, delay, elbowAngle, extenderPosition);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(owner, State.DO_DELAY, taskParams, completionEvent);
    }   //setPosition

    /**
     * This method sets the Elbow and Extender to their specifies positions.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param delay specifies the delay in seconds for starting the operation.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        String owner, double delay, Double elbowAngle, Double extenderPosition, TrcEvent completionEvent)
    {
        setPosition(owner, false, delay, elbowAngle, extenderPosition, completionEvent);
    }   //setPosition

    /**
     * This method sets the Elbow and Extender to their specifies positions.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(String owner, Double elbowAngle, Double extenderPosition, TrcEvent completionEvent)
    {
        setPosition(owner, false, 0.0, elbowAngle, extenderPosition, completionEvent);
    }   //setPosition

    /**
     * This method retracts everything.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param safeSequence specifies true to perform safe sequence so that robot won't tip over, false to do parallel.
     * @param delay specifies the delay in seconds for starting the operation.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(String owner, boolean safeSequence, double delay, TrcEvent completionEvent)
    {
        setPosition(owner, safeSequence, delay, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, completionEvent);
    }   //retract

    /**
     * This method retracts everything.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void retract(String owner, TrcEvent completionEvent)
    {
        setPosition(owner, false, 0.0, Elbow.Params.MIN_POS, Extender.Params.MIN_POS, completionEvent);
    }   //retract

    /**
     * This method cancels the ExtenderArm AutoTask.
     */
    @Override
    public void cancel()
    {
        super.cancel();
        // stopAutoTask only stop subsystems if auto task is active.
        // If subsystems are active not as part of AutoTask operation (e.g. zeroCalibrate), stopAutoTask won't do
        // anything. Let's cancel the subsystems explicitly.
        stopSubsystems(null);
    }   //cancel

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
     * operation. This is typically done before starting an auto-assist operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null ||
                        elbow.acquireExclusiveAccess(owner) &&
                        extender.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
     * operation. This is typically done if the auto-assist operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\telbowOwner=" + ownershipMgr.getOwner(elbow) +
                "\n\textenderOwner=" + ownershipMgr.getOwner(extender));
            elbow.releaseExclusiveAccess(owner);
            extender.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called by the super class to stop all the subsystems.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        elbow.cancel();
        extender.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case DO_DELAY:
                if (taskParams.delay > 0.0)
                {
                    timer.set(taskParams.delay, event);
                    sm.waitForSingleEvent(event, State.SET_POSITION);
                }
                else
                {
                    sm.setState(State.SET_POSITION);
                }
                break;

            case SET_POSITION:
                elbowEvent.clear();
                extenderEvent.clear();
                sm.setState(taskParams.safeSequence? State.RETRACT_EXTENDER: State.SET_ELBOW_ANGLE);
                break;

            case RETRACT_EXTENDER:
                if (taskParams.elbowAngle != null &&
                    Math.abs(extender.getPosition() - Extender.Params.MIN_POS) >
                    Extender.Params.POS_PID_TOLERANCE)
                {
                    // We are setting the elbow angle and the extender is extended, retract it first.
                    extender.setPosition(
                        owner, 0.0, Extender.Params.MIN_POS, true, Extender.Params.POWER_LIMIT, extenderEvent, 0.0);
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
                        owner, 0.0, taskParams.elbowAngle, true, Elbow.Params.POWER_LIMIT, elbowEvent, 4.0);
                    if (taskParams.safeSequence)
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
                        owner, 0.0, taskParams.extenderPosition, true, Extender.Params.POWER_LIMIT, extenderEvent,
                        4.0);
                    if (taskParams.safeSequence)
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
                if (taskParams.safeSequence)
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
