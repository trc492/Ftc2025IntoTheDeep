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
        PICKUP_SAMPLE,
        RAISE_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final Vision.SampleType sampleType;
        final boolean useVision;
        final Double wristRotatePos;

        TaskParams(Vision.SampleType sampleType, boolean useVision, Double wristRotatePos)
        {
            this.sampleType = sampleType;
            this.useVision = useVision;
            this.wristRotatePos = wristRotatePos;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "sampleType=" + sampleType +
                   ",useVision=" + useVision +
                   ",wristRotatePos=" + wristRotatePos;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent event;
    private final TrcEvent armEvent;

    private TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> sampleInfo = null;
    private TrcPose2D samplePose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupFromGround(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        event = new TrcEvent(moduleName);
        armEvent = new TrcEvent(moduleName + ".armEvent");
    }   //TaskAutoPickupFromGround

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param useVision specifies true to use vision to locate sample, false otherwise.
     * @param wristRotatePos specifies differential wrist rotate position, null if no change.
     */
    public void autoPickupFromGround(
        String owner, Vision.SampleType sampleType, boolean useVision, Double wristRotatePos, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(sampleType, useVision, wristRotatePos);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoPickupFromGround

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
        // ExtenderArm is an AutoTask and is not an ExclusiveSubsystem.
        return owner == null || robot.robotDrive.driveBase.acquireExclusiveAccess(owner);
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
                ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase));
            robot.robotDrive.driveBase.releaseExclusiveAccess(owner);
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
        robot.robotDrive.cancel(owner);
        robot.grabber.cancel();
        robot.extenderArm.cancel();
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
                    robot.wrist.setPosition(Wrist.Params.GROUND_PICKUP_POS, taskParams.wristRotatePos);
                    robot.extenderArm.setPosition(owner, Elbow.Params.GROUND_PICKUP_POS, null, armEvent);
                    sm.waitForSingleEvent(armEvent, nextState);
                }
                break;

            case FIND_SAMPLE:
                // Use vision to find the sample on the floor.
                sampleInfo = robot.vision.getDetectedSample(taskParams.sampleType, 0.0, -1);
                if (sampleInfo != null)
                {
                    samplePose = robot.getDetectedSamplePose(sampleInfo, true);
                    // Vision found the sample.
                    String msg = String.format(
                        Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f, rotatedAngle=%.1f",
                        taskParams.sampleType, samplePose.x, samplePose.y, samplePose.angle,
                        sampleInfo.objRotatedRectAngle);
                    tracer.traceInfo(moduleName, msg);
                    robot.speak(msg);
                    sm.setState(State.TURN_TO_SAMPLE);
                }
                else if (visionExpiredTime == null)
                {
                    // Vision doesn't find the sample, set a 1-second timeout and keep trying.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 0.75;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out and vision still not finding sample, giving up.
                    tracer.traceInfo(moduleName, "%s not found, we are done.", taskParams.sampleType);
                    sm.setState(State.DONE);
                }
                break;

            case TURN_TO_SAMPLE:
                // Vision found the sample, turn the robot toward it and use the reported rotated sample angle.
                double extenderLen = robot.getExtenderPosFromSamplePose(samplePose);
                robot.extenderArm.setPosition(owner, null, extenderLen, armEvent);
                robot.wrist.setPosition(
                    Wrist.Params.GROUND_PICKUP_POS, sampleInfo.objRotatedRectAngle * 0.8 - samplePose.angle);
                tracer.traceInfo(
                    moduleName, "samplePose=%s, extenderLen=%.1f, sampleAngle=%.1f, wristAngle=%.1f",
                    samplePose, extenderLen, sampleInfo.objRotatedRectAngle,
                    sampleInfo.objRotatedRectAngle - samplePose.angle);
                // Turning is a lot faster than extending, so just wait for extender event.
                robot.robotDrive.purePursuitDrive.start(
                    owner, null, 0.0, true, robot.robotInfo.profiledMaxVelocity,
                    robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                    new TrcPose2D(0.0, 0.0, samplePose.angle));
                sm.waitForSingleEvent(armEvent, State.PICKUP_SAMPLE);
                break;

            case PICKUP_SAMPLE:
                // Pick up sample from the floor.
                // We only care about sample color if we pick up from submersible.
                // We assume the driver would drive up to the correct sample color and orient the differential wrist
                // for picking up from ground.
                robot.grabber.autoIntake(null, 0.0, Grabber.Params.FINISH_DELAY, event, 1.0); //TO: 0.5s
                robot.elbow.setPosition(0.0,Elbow.Params.MIN_POS + 5.0, true, 0.75);
                sm.waitForSingleEvent(event, State.RAISE_ARM);
                break;

            case RAISE_ARM:
                // We may or may not get the sample. Either way, raise the arm by "fire and forget" to save time.
                robot.extenderArm.cancel();
                robot.extenderArm.setPosition(owner, Elbow.Params.GROUND_PICKUP_POS, null, null);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
//                if (robot.grabber != null)
//                {
//                    if (robot.ledIndicator != null)
//                    {
//                        // Flash the LED to show whether we got the sample and what type.
//                        robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), true);
//                    }
//                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupFromGround
