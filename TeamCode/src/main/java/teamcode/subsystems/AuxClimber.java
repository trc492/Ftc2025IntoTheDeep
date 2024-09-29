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

import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import ftclib.motor.FtcMotorActuator;
import trclib.robotcore.TrcEvent;

/**
 * This class creates the AuxClimber subsystem which consists of a climber motor actuator.
 */
class AuxClimber

{
    public final TrcMotor climber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public AuxClimber()
    {
        // Create a FtcMotorActuator for the Climber.\
        FtcMotorActuator.Params climberParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(RobotParams.ClimberParams.MOTOR_NAME, RobotParams.ClimberParams.MOTOR_TYPE, RobotParams.ClimberParams.MOTOR_INVERTED)
                .setLowerLimitSwitch(RobotParams.ClimberParams.LOWER_LIMIT_NAME, RobotParams.ClimberParams.LOWER_LIMIT_INVERTED)
                .setPositionScaleAndOffset(RobotParams.ClimberParams.INCHES_PER_COUNT, RobotParams.ClimberParams.POS_OFFSET)
                .setPositionPresets(RobotParams.ClimberParams.POS_PRESET_TOLERANCE, RobotParams.ClimberParams.posPresets);
        climber = new FtcMotorActuator(climberParams).getMotor();
    }   //AuxClimber

    /**
     * This method sets the Climber position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param pos specifies the climber position.
     * @param powerLimit specifies the maximum power limit the Climber is moving.
     * @param completionEvent specifies the event to signal when completed, can be null if not provided.
     */
    public void setClimberPosition(String owner, double pos, double powerLimit, TrcEvent completionEvent)
    {
        climber.setPosition(owner, 0.0, pos, true, powerLimit, completionEvent, 0.0);
    }   //setClimberPosition

    /**
     * This method sets the motor power to move the Climber.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the power applied to the motor.
     */
    public void setClimberPower(String owner, double power)
    {
        climber.setPower(owner, 0.0,power, 0.0, null);
    }   //setClimberPower

    /**
     * This method sets the motor power limit to move the Climber with PID control.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem, can be null if
     *        caller is not claiming ownership.
     * @param power specifies the PID power limit.
     * @param minPos specifies the lowest Climber position limit.
     * @param maxPos specifies the upper Climber position limit.
     */
    public void setClimberPidPower(String owner, double power, double minPos, double maxPos)
    {
        climber.setPidPower(owner, power, minPos, maxPos, false);
    }   //setClimberPidPower
    
    /**
     * This method returns the created Climber motor.
     *
     * @return created climber motor.
     */
    public TrcMotor getClimber()
    {
        return climber;
    }   //getClimber

}   //class AuxClimber
