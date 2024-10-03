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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements the Grabber Subsystem.
 */
public class Grabber
{
    private final Rev2mDistanceSensor rev2mSensor;
    private final TrcServoGrabber grabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Grabber()
    {
        if (RobotParams.GrabberParams.USE_REV_2M_SENSOR)
        {
            rev2mSensor = FtcOpMode.getInstance().hardwareMap.get(
                Rev2mDistanceSensor.class, RobotParams.GrabberParams.REV_2M_SENSOR_NAME);
        }
        else
        {
            rev2mSensor = null;
        }

        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
            .setPrimaryServo(
                RobotParams.GrabberParams.PRIMARY_SERVO_NAME, RobotParams.GrabberParams.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(RobotParams.GrabberParams.FOLLOWER_SERVO_NAME,
                              RobotParams.GrabberParams.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(
                RobotParams.GrabberParams.OPEN_POS, RobotParams.GrabberParams.OPEN_TIME,
                RobotParams.GrabberParams.CLOSE_POS, RobotParams.GrabberParams.CLOSE_TIME);

        if (rev2mSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                this::getSensorData, RobotParams.GrabberParams.ANALOG_TRIGGER_INVERTED,
                RobotParams.GrabberParams.SENSOR_TRIGGER_THRESHOLD, RobotParams.GrabberParams.HAS_OBJECT_THRESHOLD,
                null, false);
        }
        else if (RobotParams.GrabberParams.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(
                RobotParams.GrabberParams.DIGITAL_SENSOR_NAME, RobotParams.GrabberParams.DIGITAL_TRIGGER_INVERTED,
                null, false);
        }

        grabber = new FtcServoGrabber(RobotParams.GrabberParams.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }   //Grabber

    /**
     * This method returns the created Grabber object.
     *
     * @return created grabber object.
     */
    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

    /**
     * This method reads the analog sensor and returns its value.
     *
     * @return analog sensor value, returns zero if no analog sensor.
     */
    private double getSensorData()
    {
        if (rev2mSensor != null)
        {
            return rev2mSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }   //getSensorData

}   //class Grabber
