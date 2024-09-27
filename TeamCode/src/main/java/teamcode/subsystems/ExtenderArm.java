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

import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcExclusiveSubsystem;

/**
 * This class creates the ExtenderArm subsystem which consists of an elbow, an extender and a wrist.
 */
class ExtenderArm implements TrcExclusiveSubsystem

{
    private final String moduleName = getClass().getSimpleName();
    public final TrcDbgTrace tracer;
    public final TrcMotor elbow;
    public final TrcMotor extender;
    public final TrcMotor wrist;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ExtenderArm()
    {
        tracer = new TrcDbgTrace();
        // Create a FtcMotorActuator for the Elbow.
        elbow = null;

        // Create a FtcMotorActuator for the Extender.
        extender = null;

        // Create a FtcMotorActuator for the Wrist.
        wrist = null;
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
     * This method cancels the ExtenderArm operation if there is any, regardless of ownership.
     */
    public void cancel()
    {
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
     */
    public void zeroCalibrate(String owner)
    {
        tracer.traceInfo(moduleName, "owner=" + owner);
        cancel(owner);
        TrcEvent completionEvent = new TrcEvent(moduleName + ".zeroCalComplete");
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, tracer);

        if (validateOwnership(owner))
        {
        }
    }   //zeroCalibrate

    /**
     * This method sets the Elbow, Extender and Wrist to their specifies positions.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param elbowAngle specifies the elbow angle.
     * @param extenderPosition specifies the extender position.
     * @param wristAngle specifies the wrist angle.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setPosition(
        String owner, double elbowAngle, double extenderPosition, double wristAngle, TrcEvent completionEvent)
    {
    }   //setPosition

    //
    // Elbow methods.
    //

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
    }   //setExtenderPidPower

    //
    // Wrist methods.
    //

    /**
     * This method sets the Wrist angle.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param pos specifies the wrist position.
     * @param powerLimit specifies the maximum power limit the Extender is moving.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setWristAngle(String owner, double pos, double powerLimit, TrcEvent completionEvent)
    {
    }   //setWristAngle

    /**
     * This method sets the motor power to move the Wrist.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the power applied to the motor.
     */
    public void setWristPower(String owner, double power)
    {
    }   //setWristPower

    /**
     * This method sets the motor power limit to move the Wrist with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the PID power limit.
     * @param minPos specifies the lowest Wrist position limit.
     * @param maxPos specifies the upper Wrist position limit.
     */
    public void setWristPidPower(String owner, double power, double minPos, double maxPos)
    {
    }   //setWristPidPower

}   //class ExtenderArm
