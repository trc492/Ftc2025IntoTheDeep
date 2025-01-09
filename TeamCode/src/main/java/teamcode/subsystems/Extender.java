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
import teamcode.RobotParams;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;

/**
 * This class creates the Extender subsystem of the Extender Arm.
 */
public class Extender
{
    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "Extender";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.DcMotor;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final double INCHES_PER_COUNT             = 0.00479555742786738354339957485092 * ((35.9 - 1.4)/35.9); // Ratio Added
        public static final double POS_OFFSET                   = 15.125; // 14.75
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.75;

        // Movement Limit Constants
        // Pivot Y offset from robot center in inches
        public static final double PIVOT_Y_OFFSET               = -(408.0 * TrcUtil.INCHES_PER_MM - 9.0);
        public static final double PICKUP_POS_WRIST_OFFSET      = 1.0;  // pickup position offset from wrist joint 1.85

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 35.5; // 35.0
        public static final double START_POS                    = MIN_POS;
        public static final double HORIZONTAL_LIMIT             = RobotParams.Robot.HORIZONTAL_EXPANSION_LIMIT - 9.0;
        public static final double SPECIMEN_PICKUP_POS          = 18.0; // TODO: NEEDS TUNING
        public static final double LOW_BASKET_SCORE_POS         = MIN_POS;
        public static final double HIGH_BASKET_SCORE_POS        = 35.0;
        public static final double LOW_CHAMBER_SCORE_POS        = 17.5; //18.5
        public static final double HIGH_CHAMBER_SCORE_POS       = 17.3; //19.0
        public static final double PRE_CLIMB_POS                = 25.0;
        public static final double ASCENT_LEVEL1_POS            = MAX_POS;
        public static final double ASCENT_LEVEL2_POS            = 20.0;
        public static final double[] posPresets                 = {MIN_POS, 20.0, 25.0, 30.0, 35.0};
        public static final double POS_PRESET_TOLERANCE         = 3.0;

        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.5, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double SPRING_COMP_MAX_POWER        = -0.15;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    public final TrcMotor extender;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Extender()
    {
        FtcMotorActuator.Params extenderParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        extender = new FtcMotorActuator(extenderParams).getMotor();
        extender.setSoftwarePidEnabled(true);
        extender.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE);
        // There is no lower limit switch, so we do zero calibration by motor stall.
        extender.setStallProtection(
            Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
        extender.setPidStallDetectionEnabled(Params.STALL_RESET_TIMEOUT, Params.STALL_TIMEOUT, Params.STALL_TOLERANCE);
        // We may extend it beyond its upper limit and we don't have physical upper limit switch, so set soft limits.
        extender.setSoftPositionLimits(Extender.Params.MIN_POS, Extender.Params.MAX_POS, false);
        extender.setPositionPidPowerComp(this::getExtenderPowerComp);
        extender.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, false, false, null);
    }   //Extender

    /**
     * This method returns the created extender motor.
     *
     * @return created extender motor.
     */
    public TrcMotor getMotor()
    {
        return extender;
    }   //getMotor

    /**
     * This method is called to compute the power compensation to counteract the spring on the Extender. That's why
     * the compensation power is negative. It's not really compensating for gravity.
     *
     * @param currPower specifies the current motor power (not used).
     * @return compensation power for the extender.
     */
    private double getExtenderPowerComp(double currPower)
    {
        return Params.SPRING_COMP_MAX_POWER;
    }   //getExtenderPowerComp

}   //class Extender
