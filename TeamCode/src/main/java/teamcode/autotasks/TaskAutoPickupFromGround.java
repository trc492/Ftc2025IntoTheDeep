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

import teamcode.Robot;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Wrist;
import teamcode.subsystems.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements auto-assist task to pick up a sample from ground.
 */
public class TaskAutoPickupFromGround extends TrcAutoTask<TaskAutoPickupFromGround.State>
{
    private static final String moduleName = TaskAutoPickupFromGround.class.getSimpleName();

    public enum State
    {
        START,
        FIND_SAMPLE,
        TURN_TO_SAMPLE,
        PICKUP_SAMPLE,
        RAISE_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final Vision.SampleType sampleType;
        final boolean useVision;
        final boolean noDrive;
        TaskParams(Vision.SampleType sampleType, boolean useVision, boolean noDrive)
        {
            this.sampleType = sampleType;
            this.useVision = useVision;
            this.noDrive = noDrive;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "sampleType=" + sampleType +
                   ",useVision=" + useVision +
                   ",noDrive=" + noDrive;
        }   //toString
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent armEvent;

    private String currOwner = null;
    private TrcPose2D samplePose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupFromGround(String ownerName, Robot robot)
    {
        super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.ownerName = ownerName;
        this.robot = robot;
        event = new TrcEvent(moduleName);
        armEvent = new TrcEvent(moduleName + ".armEvent");
    }   //TaskAutoPickupFromGround

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param useVision specifies true to use vision to locate sample, false otherwise.
     * @param noDrive specifies true to not drive the robot to the sample, false otherwise.
     */
    public void autoPickupFromGround(
        Vision.SampleType sampleType, boolean useVision, boolean noDrive, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(sampleType, useVision, noDrive);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(State.START, taskParams, completionEvent);
    }   //autoPickupFromGround

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
        State nextState;

        switch (state)
        {
            case START:
                // Prep subsystems for pickup.
                if (robot.extenderArm == null || robot.grabber == null)
                {
                    // Arm or grabber don't exist, nothing we can do.
                    tracer.traceInfo(moduleName, "Arm or grabber doesn't exist, we are done.");
                    sm.setState(State.DONE);
                }
                else
                {
                    nextState =
                        taskParams.useVision && robot.vision != null &&
                        robot.vision.isSampleVisionEnabled(taskParams.sampleType)?
                            State.FIND_SAMPLE: State.PICKUP_SAMPLE;
                    robot.extenderArm.setPosition(
                        Elbow.Params.GROUND_PICKUP_POS, null, Wrist.Params.GROUND_PICKUP_POS, armEvent);
                    sm.waitForSingleEvent(armEvent, nextState);
                }
                break;

            case FIND_SAMPLE:
                // Use vision to find the sample on the floor.
                samplePose = robot.getDetectedSamplePose(taskParams.sampleType, 0.0, true);
                if (samplePose != null)
                {
                    // Vision found the sample.
                    String msg = String.format(
                        Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f",
                        taskParams.sampleType, samplePose.x, samplePose.y, samplePose.angle);
                    tracer.traceInfo(moduleName, msg);
                    robot.speak(msg);
                    sm.setState(State.TURN_TO_SAMPLE);
                }
                else if (visionExpiredTime == null)
                {
                    // Vision doesn't find the sample, set a 1-second timeout and keep trying.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out and vision still not finding sample, giving up.
                    tracer.traceInfo(moduleName, "%s not found, we are done.", taskParams.sampleType);
                    sm.setState(State.DONE);
                }
                break;

            case TURN_TO_SAMPLE:
                // Vision found the sample, turn the robot toward it.
                double extenderLen = robot.getExtenderPosFromSamplePose(samplePose);
                tracer.traceInfo(moduleName, "samplePose=%s, extenderLen=%.1f", samplePose, extenderLen);
                if (!taskParams.noDrive)
                {
                    // Turning is a lot faster than extending, so just wait for extender event.
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, null, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        new TrcPose2D(0.0, 0.0, samplePose.angle));
                }
                robot.extenderArm.setPosition(null, extenderLen, null, armEvent);
                sm.waitForSingleEvent(armEvent, State.PICKUP_SAMPLE);
                break;

            case PICKUP_SAMPLE:
                // Pick up sample from the floor.
                // We only care about sample color if we pick up from submersible.
                // We assume the driver would drive up to the correct sample color for picking up from ground.
                robot.grabber.autoIntake(null, 0.0, Grabber.Params.FINISH_DELAY, event, 2.0);
                sm.addEvent(event);
                robot.extenderArm.setPosition(Elbow.Params.MIN_POS + 4.0, null, null, armEvent);
                sm.addEvent(armEvent);
                sm.waitForEvents(State.RAISE_ARM, false, 4.0);
                break;

            case RAISE_ARM:
                // We may or may not get the sample. Either way, raise the arm by "fire and forget" to save time.
                robot.extenderArm.cancel();
                robot.extenderArm.setPosition(Elbow.Params.GROUND_PICKUP_POS, null, null, null);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
                if (robot.grabber != null)
                {
                    if (robot.ledIndicator != null)
                    {
                        // Flash the LED to show whether we got the sample and what type.
                        robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), true);
                    }
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupFromGround
