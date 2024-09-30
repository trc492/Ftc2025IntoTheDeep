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

package teamcode.subsystems;

import androidx.annotation.NonNull;

import ftclib.motor.FtcMotorActuator;
import ftclib.motor.FtcServoActuator;
import teamcode.RobotParams;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class creates the ExtenderArm subsystem which consists of an elbow, an extender and a wrist.
 */
public class ExtenderArm implements TrcExclusiveSubsystem
{
    private final String moduleName = getClass().getSimpleName();

    private enum State
    {
        RETRACT_EXTENDER,
        SET_ELBOW_ANGLE,
        SET_EXTENDER_POSITION,
        DONE
    }   //enum State

    private static class ZeroCalCallbackContext
    {
        boolean elbowZeroCalCompleted = false;
        boolean extenderZeroCalCompleted = false;
    }   //class ZeroCalCallbackContext

    public final TrcDbgTrace tracer;
    public final TrcMotor elbow;
    public final TrcMotor extender;
    public final TrcServo wrist;
    private final TrcTaskMgr.TaskObject extenderArmTask;
    private final TrcStateMachine<State> sm;
    private final TrcEvent elbowEvent;
    private final TrcEvent extenderEvent;
    private TrcEvent completionEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ExtenderArm()
    {
        tracer = new TrcDbgTrace();

        // Create a FtcMotorActuator for the Elbow.
        if (RobotParams.Preferences.useElbow)
        {
            FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(
                    RobotParams.ElbowParams.PRIMARY_MOTOR_NAME, RobotParams.ElbowParams.PRIMARY_MOTOR_TYPE,
                    RobotParams.ElbowParams.PRIMARY_MOTOR_INVERTED)
                .setFollowerMotor(
                    RobotParams.ElbowParams.FOLLOWER_MOTOR_NAME, RobotParams.ElbowParams.FOLLOWER_MOTOR_TYPE,
                    RobotParams.ElbowParams.FOLLOWER_MOTOR_INVERTED)
                .setLowerLimitSwitch(
                    RobotParams.ElbowParams.LOWER_LIMIT_NAME, RobotParams.ElbowParams.LOWER_LIMIT_INVERTED)
                .setUpperLimitSwitch(
                    RobotParams.ElbowParams.UPPER_LIMIT_NAME, RobotParams.ElbowParams.UPPER_LIMIT_INVERTED)
                .setPositionScaleAndOffset(
                    RobotParams.ElbowParams.DEG_SCALE, RobotParams.ElbowParams.POS_OFFSET,
                    RobotParams.ElbowParams.ZERO_OFFSET)
                .setPositionPresets(RobotParams.ElbowParams.POS_PRESET_TOLERANCE, RobotParams.ElbowParams.posPresets);
            elbow = new FtcMotorActuator(elbowParams).getMotor();
            elbow.setPositionPidParameters(
                RobotParams.ElbowParams.posPidCoeffs, RobotParams.ElbowParams.POS_PID_TOLERANCE);
            elbow.setPositionPidPowerComp(this::getElbowPowerComp);
            elbow.setPidStallDetectionEnabled(
                RobotParams.ElbowParams.STALL_RESET_TIMEOUT, RobotParams.ElbowParams.STALL_TIMEOUT,
                RobotParams.ElbowParams.STALL_TOLERANCE);
            elbow.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
        }
        else
        {
            elbow = null;
        }

        // Create a FtcMotorActuator for the Extender.
        if (RobotParams.Preferences.useExtender)
        {
            FtcMotorActuator.Params extenderParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(
                    RobotParams.ExtenderParams.MOTOR_NAME, RobotParams.ExtenderParams.MOTOR_TYPE,
                    RobotParams.ExtenderParams.MOTOR_INVERTED)
                .setLowerLimitSwitch(
                    RobotParams.ExtenderParams.LOWER_LIMIT_NAME, RobotParams.ExtenderParams.LOWER_LIMIT_INVERTED)
                .setPositionScaleAndOffset(
                    RobotParams.ExtenderParams.INCHES_PER_COUNT, RobotParams.ExtenderParams.POS_OFFSET)
                .setPositionPresets(
                    RobotParams.ExtenderParams.POS_PRESET_TOLERANCE, RobotParams.ExtenderParams.posPresets);
            extender = new FtcMotorActuator(extenderParams).getMotor();
            extender.setSoftwarePidEnabled(true);
            extender.setPositionPidParameters(
                RobotParams.ExtenderParams.posPidCoeffs, RobotParams.ExtenderParams.POS_PID_TOLERANCE);
            extender.setPidStallDetectionEnabled(
                RobotParams.ExtenderParams.STALL_RESET_TIMEOUT, RobotParams.ExtenderParams.STALL_TIMEOUT,
                RobotParams.ExtenderParams.STALL_TOLERANCE);
            extender.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
//            extender.resetPositionOnLowerLimitSwitch();
        }
        else
        {
            extender = null;
        }

        // Create a FtcMotorActuator for the Wrist.
        if (RobotParams.Preferences.useWrist)
        {
            FtcServoActuator.Params wristParams = new FtcServoActuator.Params()
                .setPrimaryServo(
                    RobotParams.WristParams.PRIMARY_SERVO_NAME, RobotParams.WristParams.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(
                    RobotParams.WristParams.FOLLOWER_SERVO_NAME, RobotParams.WristParams.FOLLOWER_SERVO_INVERTED);

            wrist = new FtcServoActuator(wristParams).getServo();
//            wrist.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.INFO);
        }
        else
        {
            wrist = null;
        }

        extenderArmTask = TrcTaskMgr.createTask(moduleName + ".task", this::extenderArmTask);
        sm = new TrcStateMachine<>(moduleName + ".sm");
        elbowEvent = new TrcEvent(RobotParams.ElbowParams.SUBSYSTEM_NAME);
        extenderEvent = new TrcEvent(RobotParams.ExtenderParams.SUBSYSTEM_NAME);
    }   //ExtenderArm

    /**
     * This method returns the module name.
     *
     * @return module name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return moduleName;
    }   //toString

    /**
     * This method enables/disables the Extender Arm Task.
     *
     * @param enabled specifies true to enable task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        boolean taskEnabled = extenderArmTask.isRegistered();

        if (enabled && !taskEnabled)
        {
            // Enabling task.
            sm.start(State.RETRACT_EXTENDER);
            extenderArmTask.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        }
        else if (!enabled && taskEnabled)
        {
            // Disabling task;
            sm.stop();
            extenderArmTask.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method is called to finish the operation. The operation can be finished either because it has been
     * completed or it has been canceled.
     *
     * @param completed specified true if operation has been completed, false if canceled.
     */
    private void finish(boolean completed)
    {
        setTaskEnabled(false);
        if (completionEvent != null)
        {
            if (completed)
            {
                completionEvent.signal();
            }
            else
            {
                completionEvent.cancel();
            }
            completionEvent = null;
        }
    }   //finish

    /**
     * This method is called periodically at the appropriate time this task is registered for.
     *
     * @param taskType specifies the type of task being run. This may be useful for handling multiple task types.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void extenderArmTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            switch (state)
            {
                case RETRACT_EXTENDER:
                    break;

                case SET_ELBOW_ANGLE:
                    break;

                case SET_EXTENDER_POSITION:
                    break;

                case DONE:
                default:
                    finish(true);
            }
        }
    }   //extenderArmTask

    /**
     * This method cancels the ExtenderArm operation if there is any, regardless of ownership.
     */
    public void cancel()
    {
        tracer.traceInfo(moduleName, "Canceling ...");

        if (elbow != null)
        {
            // Cancel elbow operation if any.
            elbow.cancel();
        }

        if (extender != null)
        {
            // Cancel extender operation if any.
            extender.cancel();
        }

        if (wrist != null)
        {
            // Cancel wrist operation if any.
            wrist.cancel();
        }

        finish(false);
    }   //cancel

    /**
     * This method cancels the ExtenderArm operation if there is any. This checks the ownership before canceling
     * the operation. It won't cancel if the caller does not have ownership.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not aware of ownership.
     */
    public void cancel(String owner)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        if (hasOwnership(owner))
        {
            cancel();
        }
    }   //cancel

    /**
     * This method returns the Elbow, Extender and Wrist to their resting position and zero calibrates their encoders.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        // Cancel previous operation if any.
        cancel(owner);
        // Acquires ownership on behalf of the caller if necessary.
        this.completionEvent = acquireOwnership(owner, completionEvent, tracer);
        if (this.completionEvent == null)
        {
            this.completionEvent = completionEvent;
        }

        if (validateOwnership(owner))
        {
            // Elbow and Extender zero calibration need to signal separate events which would trigger a callback to
            // check if both zero calibration have been completed. If so, it will then signal the caller's completion
            // event.
            ZeroCalCallbackContext zeroCalCallbackContext = null;
            if (this.completionEvent != null)
            {
                zeroCalCallbackContext = new ZeroCalCallbackContext();
            }

            if (elbow != null)
            {
                // Signal ZeroCal event only if there is a completion event.
                TrcEvent elbowZeroCalEvent = this.completionEvent != null? elbowEvent: null;
                if (elbowZeroCalEvent != null)
                {
                    elbowZeroCalEvent.setCallback(this::elbowZeroCalCallback, zeroCalCallbackContext);
                }
                elbow.zeroCalibrate(owner, RobotParams.ElbowParams.ZERO_CAL_POWER, elbowZeroCalEvent);
            }

            if (extender != null)
            {
                // Signal ZeroCal event only if there is a completion event.
                TrcEvent extenderZeroCalEvent = this.completionEvent != null? extenderEvent: null;
                if (extenderZeroCalEvent != null)
                {
                    extenderZeroCalEvent.setCallback(this::extenderZeroCalCallback, zeroCalCallbackContext);
                }
                extender.zeroCalibrate(owner, RobotParams.ExtenderParams.ZERO_CAL_POWER, extenderZeroCalEvent);
            }
        }
    }   //zeroCalibrate

    /**
     * This method is called when elbow zero calibration is done.
     *
     * @param context specifies the ZeroCalCallback context object.
     */
    private void elbowZeroCalCallback(Object context)
    {
        ZeroCalCallbackContext callbackContext = (ZeroCalCallbackContext) context;

        callbackContext.elbowZeroCalCompleted = true;
        if (callbackContext.extenderZeroCalCompleted)
        {
            finish(true);
        }
    }   //elbowZeroCalCallback

    /**
     * This method is called when extender zero calibration is done.
     *
     * @param context specifies the ZeroCalCallback context object.
     */
    private void extenderZeroCalCallback(Object context)
    {
        ZeroCalCallbackContext callbackContext = (ZeroCalCallbackContext) context;

        callbackContext.extenderZeroCalCompleted = true;
        if (callbackContext.elbowZeroCalCompleted)
        {
            finish(true);
        }
    }   //extenderZeroCalCallback

    /**
     * This method sets the Elbow, Extender and Wrist to their specifies positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param elbowAngle specifies the elbow angle, null if not moving elbow.
     * @param extenderPosition specifies the extender position, null if not moving extender.
     * @param wristPosition specifies the wrist position, null if not moving wrist.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        String owner, Double elbowAngle, Double extenderPosition, Double wristPosition, TrcEvent completionEvent)
    {
        // Pseudocode:
        // If moving wrist, set wrist position.
        // If not moving elbow
        //    if moving extender, set extender position
        // else if extender is retracted
        //    set elbow angle
        // else
        //    start a state machine that will do the following sequence:
        //      fully retract the extender
        //      move elbow to the set angle
        //      move extender to the set position
        setElbowAngle(owner, elbowAngle, RobotParams.ElbowParams.POWER_LIMIT, null);
        setExtenderPosition(owner, extenderPosition, RobotParams.ExtenderParams.POWER_LIMIT, null);
        setWristPosition(owner, 0.0, wristPosition, null, 0.0);
    }   //setPosition

    //
    // Elbow methods.
    //

    /**
     * This method is called to compute the power compensation to counteract gravity on the Elbow.
     *
     * @param currPower specifies the current motor power (not used).
     * @return gravity compensation for the arm.
     */
    private double getElbowPowerComp(double currPower)
    {
        return RobotParams.ElbowParams.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(elbow.getPosition()));
    }   //getElbowPowerComp

    /**
     * This method sets the Elbow angle.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param pos specifies the elbow angle.
     * @param powerLimit specifies the maximum power limit the Elbow is moving.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setElbowAngle(String owner, double pos, double powerLimit, TrcEvent completionEvent)
    {
        if (elbow != null)
        {
            elbow.setPosition(owner, 0.0, pos, true, powerLimit, completionEvent, 0.0);
        }
    }   //setElbowAngle

    /**
     * This method sets the motor power to move the Elbow.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the power applied to the motor.
     */
    public void setElbowPower(String owner, double power)
    {
        if (elbow != null)
        {
            elbow.setPower(owner, 0.0, power, 0.0, null);
        }
    }   //setElbowPower

    /**
     * This method sets the motor power limit to move the Elbow with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the PID power limit.
     * @param minPos specifies the lowest Elbow position limit.
     * @param maxPos specifies the upper Elbow position limit.
     */
    public void setElbowPidPower(String owner, double power, double minPos, double maxPos)
    {
        if (elbow != null)
        {
            elbow.setPidPower(owner, power, minPos, maxPos, false);
        }
    }   //setElbowPidPower

    //
    // Extender methods.
    //

    /**
     * This method sets the Extender position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param pos specifies the extender position.
     * @param powerLimit specifies the maximum power limit the Extender is moving.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setExtenderPosition(String owner, double pos, double powerLimit, TrcEvent completionEvent)
    {
        if (extender != null)
        {
            extender.setPosition(owner, 0.0, pos, true, powerLimit, completionEvent, 0.0);
        }
    }   //setExtenderPosition

    /**
     * This method sets the motor power to move the Extender.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the power applied to the motor.
     */
    public void setExtenderPower(String owner, double power)
    {
        if (extender != null)
        {
            extender.setPower(owner, 0.0, power, 0.0, null);
        }
    }   //setExtenderPower

    /**
     * This method sets the motor power limit to move the Extender with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the PID power limit.
     * @param minPos specifies the lowest Extender position limit.
     * @param maxPos specifies the upper Extender position limit.
     */
    public void setExtenderPidPower(String owner, double power, double minPos, double maxPos)
    {
        if (extender != null)
        {
            extender.setPidPower(owner, power, minPos, maxPos, false);
        }
    }   //setExtenderPidPower

    //
    // Wrist methods.
    //

    /**
     * This method sets the wrist position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *        setPhysicalPosRange is called with the degree range.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void setWristPosition(String owner, double delay, double position, TrcEvent completionEvent, double timeout)
    {
        if (wrist != null)
        {
            wrist.setPosition(owner, delay, position, completionEvent, timeout);
        }
    }   //wristSetPosition

    /**
     * This method sets the wrist position.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     */
    public void setWristPosition(double position)
    {
        setWristPosition(null, 0.0, position, null, 0.0);
    }   //wristSetPosition

}   //class ExtenderArm
