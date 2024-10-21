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

import ftclib.motor.FtcMotorActuator;
import teamcode.Robot;
import teamcode.params.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the Elbow subsystem of the Extender Arm.
 */
public class Elbow
{
    /**
     * This class contains Elbow subsystem constants and parameters.
     */
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "Elbow";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final boolean LOWER_LIMIT_INVERTED        = true;

        public static final double ENCODER_CPR                  = RobotParams.Gobilda.MOTOR_5203_84_ENC_PPR;
        public static final double GEAR_RATIO                   = 44.0 / 10.0;
        public static final double DEG_SCALE                    = 360.0 / (ENCODER_CPR * GEAR_RATIO);
        public static final double POS_OFFSET                   = 0.0;
        public static final double ZERO_OFFSET                  = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.2;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 120.0;
        public static final double GROUND_PICKUP_POS            = MIN_POS;
        public static final double SPECIMEN_PICKUP_POS          = 30; // TODO: NEEDS TUNING
        public static final double[] posPresets                 = {MIN_POS, 30.0, 60.0, 90.0, MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.12, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.2/Extender.Params.POS_OFFSET;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    private final Robot robot;
    public final TrcMotor elbow;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elbow(Robot robot)
    {
        this.robot = robot;
        FtcMotorActuator.Params elbowParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMIT_NAME, Params.LOWER_LIMIT_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_SCALE, Params.POS_OFFSET, Params.ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        elbow = new FtcMotorActuator(elbowParams).getMotor();
        elbow.setSoftwarePidEnabled(true);
        elbow.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE);
        elbow.setPositionPidPowerComp(this::getElbowPowerComp);
        // Elbow has a large gear ratio,so unlikely to have PID hang. No need to do PID stall detection.
//        elbow.setPidStallDetectionEnabled(Params.STALL_RESET_TIMEOUT, Params.STALL_TIMEOUT, Params.STALL_TOLERANCE);
        elbow.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
    }   //Elbow

    /**
     * This method returns the created elbow motor.
     *
     * @return created elbow motor.
     */
    public TrcMotor getMotor()
    {
        return elbow;
    }   //getMotor

    /**
     * This method is called to compute the power compensation to counteract gravity on the Elbow.
     *
     * @param currPower specifies the current motor power (not used).
     * @return gravity compensation for the arm.
     */
    private double getElbowPowerComp(double currPower)
    {
        if (robot.extender != null)
        {
            final double elbowLength = 4.1695;  // from CAD model.
            double extenderLength = robot.extender.getPosition();
            double extenderAngleRadian = Math.toRadians(elbow.getPosition()) - Math.atan(elbowLength/extenderLength);
            // Adjust extender length to be the length to the pivot point instead of the base point.
            extenderLength = TrcUtil.magnitude(elbowLength, extenderLength);
            // Extender angle is zero horizontal.
            return Params.GRAVITY_COMP_MAX_POWER*extenderLength*Math.cos(extenderAngleRadian);
        }
        else
        {
            // We should not be calling gravity comp if extender has not been created.
            // But to avoid NullPointerException just in case, return 0.0 gravity comp power.
            return 0.0;
        }
    }   //getElbowPowerComp

}   //class Elbow
