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

/**
 * This class creates the AuxClimber subsystem which consists of a climber motor actuator.
 */
public class AuxClimber

{
    public final TrcMotor climber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public AuxClimber()
    {
        // Create a FtcMotorActuator for the Climber.
        FtcMotorActuator.Params climberParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                RobotParams.ClimberParams.MOTOR_NAME, RobotParams.ClimberParams.MOTOR_TYPE,
                RobotParams.ClimberParams.MOTOR_INVERTED)
            .setLowerLimitSwitch(
                RobotParams.ClimberParams.LOWER_LIMIT_NAME, RobotParams.ClimberParams.LOWER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(
                RobotParams.ClimberParams.INCHES_PER_COUNT, RobotParams.ClimberParams.POS_OFFSET)
            .setPositionPresets(
                RobotParams.ClimberParams.POS_PRESET_TOLERANCE, RobotParams.ClimberParams.posPresets);
        climber = new FtcMotorActuator(climberParams).getMotor();
    }   //AuxClimber

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
