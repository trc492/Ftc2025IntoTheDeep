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

import java.util.Locale;

import teamcode.Robot;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Wrist;
import teamcode.vision.Vision;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

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
        PICK_UP_SAMPLE,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final Vision.SampleType sampleType;
        TaskParams(Vision.SampleType sampleType)
        {
            this.sampleType = sampleType;
        }   //TaskParams
    }   //class TaskParams

    private final String ownerName;
    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent grabberEvent;

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
        grabberEvent = new TrcEvent(Grabber.Params.SUBSYSTEM_NAME);
    }   //TaskAutoPickupFromGround

    /**
     * This method starts the auto-assist operation.
     *
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupFromGround(Vision.SampleType sampleType, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "sampleType=" + sampleType + ",event=" + completionEvent);
        startAutoTask(State.START, new TaskParams(sampleType), completionEvent);
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
            case START:
                samplePose = null;
                if (robot.vision != null)
                {
                    // Prep for pickup.
                    robot.vision.setSampleVisionEnabled(taskParams.sampleType, true);
                    if (robot.extenderArm != null)
                    {
                        robot.extenderArm.setPosition(
                            Elbow.Params.GROUND_PICKUP_POS, Extender.Params.MIN_POS, Wrist.Params.GROUND_PICKUP_POS,
                            event);
                        sm.waitForSingleEvent(event, State.FIND_SAMPLE);
                    }
                    else
                    {
                        sm.setState(State.FIND_SAMPLE);
                    }
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case FIND_SAMPLE:
                // need to tune all of this based on this year's game
                if (robot.vision.isSampleVisionEnabled(taskParams.sampleType))
                {
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo =
                        robot.vision.getDetectedSample(taskParams.sampleType, -1);
                    if (sampleInfo != null)
                    {
                        samplePose = sampleInfo.objPose;
                        String msg = String.format(
                            Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f",
                            sampleInfo.detectedObj.label, sampleInfo.objPose.x, sampleInfo.objPose.y,
                            sampleInfo.objPose.angle);
                        tracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                        sm.setState(State.TURN_TO_SAMPLE);
                    }
                    else if (visionExpiredTime == null)
                    {
                        visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                    }
                    else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                    {
                        tracer.traceInfo(moduleName, "%s not found.", taskParams.sampleType);
                        sm.setState(State.DONE);
                    }
                }
                else
                {
                    tracer.traceInfo(moduleName, "Vision not enabled.");
                    sm.setState(State.DONE);
                }
                break;

            case TURN_TO_SAMPLE:
                if (samplePose != null)
                {
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, event, 0.0, robot.robotDrive.driveBase.getFieldPosition(), true,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        new TrcPose2D(0.0, 0.0, samplePose.angle));
                    sm.waitForSingleEvent(event, State.PICK_UP_SAMPLE);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            case PICK_UP_SAMPLE:
                if (robot.extenderArm != null && robot.grabber != null)
                {
                    // We only care about sample color if we pick up from submersible.
                    // We assume the driver would drive up to the correct sample color for picking up from ground.
                    robot.grabber.autoIntake(currOwner, 0.0, grabberEvent);
                    sm.addEvent(grabberEvent);
                    robot.extenderArm.setPosition(null, samplePose.y, null, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE, false);
                }
                else
                {
                    sm.setState(State.DONE);
                }
                break;

            default:
            case DONE:
                // Stop task.
                if (robot.grabber != null)
                {
                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), false);
                    }
                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickupFromGround
