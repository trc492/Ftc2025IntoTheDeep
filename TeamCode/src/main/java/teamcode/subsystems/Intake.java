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
import ftclib.sensor.FtcDistanceSensor;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcSensor;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerThresholdZones;

public class Intake
{
    private final TrcDbgTrace tracer;
    public final TrcMotor intake;
    private final FtcDistanceSensor intakeSensor;
    private final TrcTriggerThresholdZones analogTrigger;
    private TrcEvent completionEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     */

    public Intake()
    {
        this.tracer = new TrcDbgTrace();

        FtcMotorActuator.Params intakeParams = new FtcMotorActuator.Params()
            .setPrimaryMotor(
                RobotParams.Intake.PRIMARY_MOTOR_NAME, RobotParams.Intake.PRIMARY_MOTOR_TYPE,
                RobotParams.Intake.PRIMARY_MOTOR_INVERTED)
            .setFollowerMotor(
                RobotParams.Intake.FOLLOWER_MOTOR_NAME, RobotParams.Intake.FOLLOWER_MOTOR_TYPE,
                RobotParams.Intake.FOLLOWER_MOTOR_INVERTED);
        intake = new FtcMotorActuator(intakeParams).getMotor();

        intakeSensor = null; //new FtcDistanceSensor(RobotParams.Intake.SENSOR_NAME);
        analogTrigger = null; //new TrcTriggerThresholdZones(RobotParams.Intake.SUBSYSTEM_NAME + ".analogTrigger", this::getDistance, RobotParams.Intake.SENSOR_THRESHOLDS, false);
    }   //Intake

    public void stop()
    {
        intake.stop();
    }   //stop

    public void setOn(double delay, double power, double duration, TrcEvent event)
    {
        intake.setPower(delay, power, duration, event);
    }   //setOn

    public void setOn(double delay, double duration, TrcEvent event)
    {
        intake.setPower(delay, RobotParams.Intake.FORWARD_POWER, duration, event);
    }   //setOn

    public void setOn(boolean on)
    {
        intake.setPower(on? RobotParams.Intake.FORWARD_POWER: 0.0);
    }   //setOn

    public void setReverse(double delay, double duration, TrcEvent event)
    {
        intake.setPower(delay, RobotParams.Intake.REVERSE_POWER, duration, event);
    }   //setReverse

    public void setReverse(boolean on)
    {
        intake.setPower(on? RobotParams.Intake.REVERSE_POWER: 0.0);
        analogTrigger.disableTrigger();
    }   //setReverse

    public void pickupSample(boolean on, TrcEvent event)
    {
        setOn(on);
        if (analogTrigger != null)
        {
            if (on)
            {
                completionEvent = event;
                analogTrigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::analogTriggerCallback);
            }
            else
            {
                completionEvent = null;
                analogTrigger.disableTrigger();
            }
        }
    }   //pickupSample

    public boolean hasSample()
    {
        return getDistance() < RobotParams.Intake.SENSOR_THRESHOLDS[0];
    }   //hasSample

    /**
     * This method is called when an analog sensor threshold has been crossed.
     *
     * @param context specifies the callback context.
     */
    private void analogTriggerCallback(Object context)
    {
        TrcTriggerThresholdZones.CallbackContext callbackContext = (TrcTriggerThresholdZones.CallbackContext) context;

        tracer.traceInfo(
            RobotParams.Intake.SUBSYSTEM_NAME, "Zone=%d->%d, value=%.3f",
            callbackContext.prevZone, callbackContext.currZone, callbackContext.sensorValue);
        if (hasSample())
        {
            analogTrigger.disableTrigger();
            // CodeReview: it doesn't work this way. By calling setPower twice, the second one will cancel the first
            // one.
            intake.setPower(0.0, RobotParams.Intake.FORWARD_POWER, 1.0);
            intake.setPower(1.0, RobotParams.Intake.REVERSE_POWER, 0.5);

            if (completionEvent != null)
            {
                completionEvent.signal();
                completionEvent = null;
            }
        }
    }   //analogTriggerCallback

    /**
     * This method is called the TrcTriggerThresholdZones to get the sensor data.
     *
     * @return distance to detected object in inches.
     */
    public double getDistance()
    {
        TrcSensor.SensorData<Double> data =
            intakeSensor != null? intakeSensor.getProcessedData(0, FtcDistanceSensor.DataType.DISTANCE_INCH): null;
        return data != null && data.value != null? data.value: 500.0;
    }   //getDistance

}   //class Intake
