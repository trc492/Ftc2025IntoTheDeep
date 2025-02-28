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

import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Elbow;
import teamcode.subsystems.Extender;
import teamcode.subsystems.Grabber;
import teamcode.subsystems.Wrist;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements auto-assist task to pick up a specimen from ground.
 */
public class TaskAutoPickupSpecimen extends TrcAutoTask<TaskAutoPickupSpecimen.State>
{
    private static final String moduleName = TaskAutoPickupSpecimen.class.getSimpleName();

    public enum State
    {
        START,
        DRIVE_TO_PICKUP,
        APPROACH_SPECIMEN,
        FIND_SPECIMEN,
        ALIGN_TO_SPECIMEN,
        PICKUP_SPECIMEN,
        RETRACT_ARM,
        DONE
    }   //enum State

    private static class TaskParams
    {
        final FtcAuto.Alliance alliance;
        final boolean useVision;
        final boolean fromObservation;
        TaskParams(FtcAuto.Alliance alliance, boolean useVision, boolean fromObservation)
        {
            this.alliance = alliance;
            this.useVision = useVision;
            this.fromObservation = fromObservation;
        }   //TaskParams

        @NonNull
        public String toString()
        {
            return "alliance=" + alliance + ",useVision=" + useVision + ",noDrive=" + fromObservation;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent event;

    private TrcPose2D specimenPose = null;
    private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickupSpecimen(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        event = new TrcEvent(moduleName);
    }   //TaskAutoPickupSpecimen

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param alliance specifies the alliance color, can be null if caller is TeleOp.
     * @param useVision specifies true to use Vision, false otherwise.
     * @param fromObservation specifies true if the robot is already right in front of the specimen, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickupSpecimen(
        String owner, FtcAuto.Alliance alliance, boolean useVision, boolean fromObservation, TrcEvent completionEvent)
    {
        if (alliance == null)
        {
            // Caller is TeleOp, let's determine the alliance color by robot's location.
            // Caveat: this assumes odemetry is current in TeleOp. If odometry is not setup correctly, this would be
            // wrong. In other words, if TeleOp is run without prior Auto, the driver must do an AprilTag
            // relocalization to make odometry current before this would work.
            alliance = robot.robotDrive.driveBase.getFieldPosition().y < 0.0?
                FtcAuto.Alliance.RED_ALLIANCE: FtcAuto.Alliance.BLUE_ALLIANCE;
        }

        TaskParams taskParams = new TaskParams(alliance, useVision, fromObservation);
        tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoPickupSpecimen

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
                "\n\trobotDriveOwner=" + ownershipMgr.getOwner(robot.robotDrive.driveBase));
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

        switch (state)
        {
            case START:
                // Prep subsystems for the pickup.
                if (robot.extenderArm == null || robot.grabber == null)
                {
                    // Arm or grabber don't exist, nothing we can do.
                    tracer.traceInfo(moduleName, "Arm or grabber doesn't exist, we are done.");
                    sm.setState(State.DONE);
                }
//                else if (taskParams.fromObservation)
//                {
//                    robot.wrist.setPosition(Wrist.Params.SPECIMEN_PICKUP_POS, 0.0);
//                    robot.extenderArm.setPosition(
//                            Elbow.Params.SPECIMEN_PICKUP_POS, Extender.Params.SPECIMEN_PICKUP_POS, null);
//                    sm.setState(State.APPROACH_SPECIMEN);
//                }
                else
                {
                    // Fire and forget to save time.
                    robot.wrist.setPosition(Wrist.Params.SPECIMEN_PICKUP_POS, 0.0);
                    robot.extenderArm.setPosition(
                        owner, Elbow.Params.SPECIMEN_PICKUP_POS, Extender.Params.SPECIMEN_PICKUP_POS, null);
                    sm.setState(State.DRIVE_TO_PICKUP);
                }
                break;

            case DRIVE_TO_PICKUP:
                // Drive to the specimen pickup location.
                if (!taskParams.fromObservation)
                {
                    TrcPose2D intermediate1 = RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP.clone();
                    intermediate1.y += 2.0;
                    robot.robotDrive.purePursuitDrive.start(
                        owner, event, 5.0, false, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(intermediate1, taskParams.alliance),
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                }
                else
                {
//                    TrcPose2D intermediate1 = robot.robotDrive.driveBase.getFieldPosition();
//                    intermediate1.y -= 10.0;
//                    intermediate1.angle = 90.0;
//                    TrcPose2D intermediate2 = RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP.clone();
//                    intermediate2.y += 6.0;
//                    intermediate2.angle = 180.0;
//
//                    robot.robotDrive.purePursuitDrive.start(
//                            currOwner, event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
//                            robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
//                            robot.adjustPoseByAlliance(intermediate1,taskParams.alliance),
//                            robot.adjustPoseByAlliance(intermediate2, taskParams.alliance),
//                            robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                    robot.robotDrive.purePursuitDrive.start(
                        owner, event, 5.0, false, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        robot.adjustPoseByAlliance(RobotParams.Game.RED_OBSERVATION_ZONE_PICKUP, taskParams.alliance));
                }
                sm.waitForSingleEvent(event, taskParams.useVision? State.FIND_SPECIMEN: State.APPROACH_SPECIMEN);
                break;

            case FIND_SPECIMEN:
                // Use vision to find the sample on the wall.
                specimenPose = robot.getDetectedSamplePose(
                    Robot.sampleType, RobotParams.Game.SPECIMEN_GROUND_OFFSET, true);
                if (specimenPose != null)
                {
                    // Vision found the specimen.
                    String msg = String.format(
                        Locale.US, "%s is found at x %.1f, y %.1f, angle=%.1f",
                        Robot.sampleType, specimenPose.x, specimenPose.y, specimenPose.angle);
                    tracer.traceInfo(moduleName, msg);
                    robot.speak(msg);
                    sm.setState(State.ALIGN_TO_SPECIMEN);
                }
                else if (visionExpiredTime == null)
                {
                    // Vision doesn't find the specimen, set a 1-second timeout and keep trying.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out and vision still not finding specimen, giving up.
                    tracer.traceInfo(moduleName, "%s not found, we are done.", Robot.sampleType);
                    sm.setState(State.DONE);
                }
                break;

            case ALIGN_TO_SPECIMEN:
                // Vision found the specimen, align the robot to it.
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                double targetHeading = taskParams.alliance == FtcAuto.Alliance.RED_ALLIANCE? 180.0: 0.0;
                robot.robotDrive.purePursuitDrive.start(
                    owner, null, 0.0, true, robot.robotInfo.profiledMaxVelocity,
                    robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                    new TrcPose2D(specimenPose.x, 0.0, targetHeading - robotPose.angle));
                sm.waitForSingleEvent(event, State.APPROACH_SPECIMEN);
                break;

            case APPROACH_SPECIMEN:
                // Turn on intake and approach specimen slowly.
                robot.grabber.autoIntake(null, 0.0, Grabber.Params.FINISH_DELAY, event, 1.5);
                robot.robotDrive.driveBase.holonomicDrive(owner, 0.0, 0.3, 0.0);
                sm.waitForSingleEvent(event, State.PICKUP_SPECIMEN);
                break;

            case PICKUP_SPECIMEN:
                // Grabber got the specimen, stop the drive and raise the arm to pick it up.
                robot.robotDrive.driveBase.stop(owner);
                robot.grabber.cancel();
                robot.extenderArm.setPosition(owner, Elbow.Params.SPECIMEN_PICKUP_POS + 10.0, null, event);
                sm.waitForSingleEvent(event, State.RETRACT_ARM);
                break;

            case RETRACT_ARM:
                // Retract the arm with "fire and forget".
                // Code Review: the retract won't finish because DONE state will cancel it.
                robot.extenderArm.retract(owner, null);
                sm.setState(State.DONE);
                break;

            default:
            case DONE:
                // Stop task.
//                if (robot.grabber != null && robot.ledIndicator != null)
//                {
//                    // Flash the LED to show whether we got the specimen and what type.
//                    robot.ledIndicator.setDetectedSample(robot.grabber.getSampleType(), true);
//                }
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupSpecimen
