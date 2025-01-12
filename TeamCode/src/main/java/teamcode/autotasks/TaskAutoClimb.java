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
public class TaskAutoClimb extends TrcAutoTask<TaskAutoClimb.State>
{
    private static final String moduleName = TaskAutoClimb.class.getSimpleName();

    public enum State
    {
        LEVEL1_START,
        LEVEL1_ASCENT,
        LEVEL2_START,
        FOLD_ROBOT,
        ELBOW_TORQUE,
        ARM_RETRACT,
        ELBOW_RETRACT,
        LEVEL2_ASCENT,
        LEVEL3_START,
        DONE
    }   //enum State

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
    public void autoClimbLevel1(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL1_START, null, completionEvent);
    }   //autoClimbLevel1

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel2(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL2_START, null, completionEvent);
    }   //autoClimbLevel2

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoClimbLevel3(TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "event=" + completionEvent);
        startAutoTask(State.LEVEL3_START, null, completionEvent);
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
        // ExtenderArm is an AutoTask and is not an ExclusiveSubsystem so we don't need to acquire its ownership.
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
        // Restore PID and stall protection back to default.
        robot.extender.setPidStallDetectionEnabled(true);
        robot.extender.setPidStallDetectionEnabled(
            Extender.Params.STALL_RESET_TIMEOUT, Extender.Params.STALL_TIMEOUT, Extender.Params.STALL_TOLERANCE);
        robot.extender.setPositionPidParameters(Extender.Params.posPidCoeffs, Extender.Params.POS_PID_TOLERANCE);
        robot.elbow.setPositionPidParameters(Elbow.Params.posPidCoeffs, Elbow.Params.POS_PID_TOLERANCE);
        robot.extenderArm.cancel();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param params specifies the task parameters (not used).
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
        switch (state)
        {
            case LEVEL1_START:
                robot.extenderArm.setPosition(
                    Elbow.Params.PRE_CLIMB_POS, Extender.Params.ASCENT_LEVEL1_POS, event);
                sm.waitForSingleEvent(event, State.LEVEL1_ASCENT);
                break;

            case LEVEL1_ASCENT:
                robot.wrist.setPosition(Wrist.Params.ASCENT_LEVEL1_POS, 0.0);
                robot.extenderArm.setPosition(Elbow.Params.ASCENT_LEVEL1_POS, null, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case LEVEL2_START:
                robot.extender.setPidStallDetectionEnabled(false);
                robot.extender.setStallProtection(0.0, 0.0, 0.0, 0.0);
                robot.elbow.setPositionPidParameters(2.5, 1.0, 0.01, 0.0, Elbow.Params.POS_PID_TOLERANCE);
                robot.extender.setPositionPidParameters(3.0, 0.0, 0.01, 0.0, Extender.Params.POS_PID_TOLERANCE);

                robot.extenderArm.setPosition(Elbow.Params.LEVEL2_RETRACT_POS, null, event);
                sm.waitForSingleEvent(event, State.FOLD_ROBOT);
                break;

            case FOLD_ROBOT:
                robot.extenderArm.setPosition(null, Extender.Params.MIN_POS + 2.0, event);
                sm.waitForSingleEvent(event, State.ELBOW_TORQUE);
                break;

            case ELBOW_TORQUE:
                // Code Review: why set extender position separately and not using extenderArm? This will not work.
                robot.extenderArm.setPosition(Elbow.Params.LEVEL2_TORQUE_POS, null, event);
                robot.extender.setPosition(Extender.Params.MIN_POS, true);
                sm.waitForSingleEvent(event, State.ARM_RETRACT);
                break;

            case ARM_RETRACT:
                robot.extenderArm.setPosition(null, Extender.Params.MIN_POS, event);
                sm.waitForSingleEvent(event, State.ELBOW_RETRACT);
                break;

            case ELBOW_RETRACT:
//                robot.elbow.setPower(-1.0);
                robot.elbow.setPositionPidParameters(8, 0.0, 0.0, 0.0, Elbow.Params.POS_PID_TOLERANCE);
//                if (robot.elbow.getPosition() <= Elbow.Params.LEVEL2_FINAL_POS + 2 && robot.elbow.getPosition() >= Elbow.Params.LEVEL2_FINAL_POS - 2)
//                {
//                    sm.setState(State.LEVEL2_ASCENT);
//                }
                robot.extenderArm.setPosition(Elbow.Params.LEVEL2_FINAL_POS, null, event);
                sm.waitForSingleEvent(event, State.LEVEL2_ASCENT);
                break;

            case LEVEL2_ASCENT:
                robot.extenderArm.setPosition(null, Extender.Params.ASCENT_LEVEL2_POS, event);
                sm.waitForSingleEvent(event, State.DONE);
                break;

            case LEVEL3_START:
                // Don't have level 3 climb, hence setting state to done.
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
