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
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoClimb extends TrcAutoTask<TaskAutoClimb.State>
{
    private static final String moduleName = TaskAutoClimb.class.getSimpleName();

    public enum State
    {
        LEVEL2_START,
        PULL_UP,
        FOLD_ROBOT,
        ENGAGE_SECONDARY_HOOK,
        LEVEL3_START,
        DONE
    }   //enum State

    private static class TaskParams
    {
        TaskParams()
        {
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
    public TaskAutoClimb(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
    }   //TaskAutoClimb

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel2(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL2_START, new TaskParams(), completionEvent);
    }   //autoClimbLevel2

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel3(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL3_START, new TaskParams(), completionEvent);
    }   //autoClimbLevel3

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
        currOwner = ownerName;
        tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
        return true;
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
                "Releasing subsystem ownership (currOwner=" + currOwner + ").");
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
            case LEVEL2_START:
                robot.extenderArm.setPosition(Elbow.Params.LEVEL2_CLIMB_POS, null, null, event);
                sm.waitForSingleEvent(event, State.PULL_UP);
                break;

            case PULL_UP:
                robot.extenderArm.setPosition(null, Extender.Params.MIN_POS, null, event);
                sm.waitForSingleEvent(event, State.FOLD_ROBOT);
                break;

            case FOLD_ROBOT:
                robot.extenderArm.setPosition(Elbow.Params.LEVEL2_RETRACT_POS, null, null, event);
                sm.waitForSingleEvent(event, State.ENGAGE_SECONDARY_HOOK);
                break;

            case ENGAGE_SECONDARY_HOOK:
                robot.extenderArm.setPosition(
                    null, Extender.Params.MIN_POS + Extender.Params.LEVEL2_HOOK_POS, null, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case LEVEL3_START:
                // Don't have level 3 climb, hence setting state to done
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoClimb
