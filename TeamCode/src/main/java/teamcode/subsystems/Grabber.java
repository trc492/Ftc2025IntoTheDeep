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

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.motor.FtcMotorActuator;
import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcMotorGrabber;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerThresholdZones;
import trclib.subsystem.TrcMotorGrabber;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements the Grabber Subsystem.
 */
public class Grabber
{

    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final FtcMotorActuator.MotorType PRIMARY_MOTOR_TYPE = FtcMotorActuator.MotorType.CRServo;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;
        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final FtcMotorActuator.MotorType FOLLOWER_MOTOR_TYPE = FtcMotorActuator.MotorType.CRServo;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = !PRIMARY_MOTOR_INVERTED;

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_SERVO_INVERTED      = false;
        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        // Movement Limit Constants
        public static final double GRABBER_LENGTH               = 5.75; // Not Tuned

        public static final String SENSOR_NAME                  = SUBSYSTEM_NAME + ".sensor";
        public static final boolean SENSOR_TRIGGER_INVERTED     = true;
        public static final double SENSOR_TRIGGER_THRESHOLD     = 0.95;
        public static final double RED_THRESHOLD_LOW            = 30.0;
        public static final double RED_THRESHOLD_HIGH           = 340.0;
        public static final double YELLOW_THRESHOLD_LOW         = 60.0;
        public static final double YELLOW_THRESHOLD_HIGH        = 80.0;
        public static final double BLACK_THRESHOLD_LOW          = 160.0;
        public static final double BLACK_THRESHOLD_HIGH         = 190.0;
        public static final double BLUE_THRESHOLD_LOW           = 200.0;
        public static final double BLUE_THRESHOLD_HIGH          = 240.0;
        // Color thresholds must be sorting in ascending order.
        public static final double[] COLOR_THRESHOLDS           = new double[] {
            RED_THRESHOLD_LOW, YELLOW_THRESHOLD_LOW, YELLOW_THRESHOLD_HIGH,
            BLACK_THRESHOLD_LOW, BLACK_THRESHOLD_HIGH, BLUE_THRESHOLD_LOW,
            BLUE_THRESHOLD_HIGH, RED_THRESHOLD_HIGH};

        public static final double INTAKE_POWER                 = 1.0;
        public static final double EJECT_POWER                  = -0.225;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0; // OLD: 0.08s
        public static final double DUMP_TIME                    = 0.5; // TO: 0.45s
        public static final double DUMP_DELAY                   = 0.0; // OLD: 0.4s

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;
    }   //class Params

    private final Robot robot;
    private final RevColorSensorV3 colorSensor;
    private final TrcMotorGrabber motorGrabber;
    private final TrcServoGrabber servoGrabber;
    private Vision.SampleType sampleType;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Grabber(Robot robot)
    {
        this.robot = robot;
        colorSensor = FtcOpMode.getInstance().hardwareMap.get(RevColorSensorV3.class, Params.SENSOR_NAME);
        TrcTriggerThresholdZones colorTrigger = new TrcTriggerThresholdZones(
            Params.SUBSYSTEM_NAME + ".colorTrigger", this::getSensorHue, Params.COLOR_THRESHOLDS, false);
        colorTrigger.enableTrigger(TrcTrigger.TriggerMode.OnBoth, this::colorTriggerCallback);

        if (RobotParams.Preferences.useMotorGrabber)
        {
            FtcMotorGrabber.Params grabberParams = new FtcMotorGrabber.Params()
                .setPrimaryMotor(Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED)
                .setFollowerMotor(
                    Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED)
                .setPowerParams(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
                .setAnalogSensorTrigger(
                    this::getSensorDistance, Params.SENSOR_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);
            motorGrabber = new FtcMotorGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
            motorGrabber.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
            servoGrabber = null;
        }
        else
        {
            FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
                .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_INVERTED)
                .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME)
                .setAnalogSensorTrigger(
                    this::getSensorDistance, Params.SENSOR_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);
            servoGrabber = new FtcServoGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
            servoGrabber.open();
            motorGrabber = null;
        }
    }   //Grabber

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return motorGrabber != null? motorGrabber.toString(): servoGrabber.toString();
    }   //toString

    /**
     * This method returns the created MotorGrabber object.
     *
     * @return created grabber object.
     */
    public TrcMotorGrabber getMotorGrabber()
    {
        return motorGrabber;
    }   //getMotorGrabber

    /**
     * This method returns the created ServoGrabber object.
     *
     * @return created grabber object.
     */
    public TrcServoGrabber getServoGrabber()
    {
        return servoGrabber;
    }   //getServoGrabber

    /**
     * This method acquires exclusive ownership of the subsystem if it's not already owned by somebody else.
     *
     * @param owner specifies the ID string of the caller requesting ownership.
     * @return true if successfully acquired ownership, false otherwise.
     */
    public boolean acquireExclusiveAccess(String owner)
    {
        return motorGrabber != null?
            motorGrabber.acquireExclusiveAccess(owner): servoGrabber.acquireExclusiveAccess(owner);
    }   //acquireExclusiveAccess

    /**
     * This method release exclusive ownership of the subsystem if the caller is indeed the owner.
     *
     * @param owner specifies the ID string of the caller releasing ownership.
     * @return true if successfully releasing ownership, false otherwise.
     */
    public boolean releaseExclusiveAccess(String owner)
    {
        return motorGrabber != null?
            motorGrabber.releaseExclusiveAccess(owner): servoGrabber.releaseExclusiveAccess(owner);
    }   //releaseExclusiveAccess

    /**
     *
     * This method checks if object is detected.
     *
     * @return true if object is detected, false otherwise.
     */
    public boolean hasObject()
    {
        return motorGrabber != null? motorGrabber.hasObject(): servoGrabber.hasObject();
    }   //hasObject

    /**
     * This method returns the current owner of this subsystem.
     *
     * @return current owner, null if no owner.
     */
    public String getOwner()
    {
        TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
        return motorGrabber != null? ownershipMgr.getOwner(motorGrabber): ownershipMgr.getOwner(servoGrabber);
    }   //getOwner

    /**
     * This method checks if auto operation is active.
     *
     * @return true if auto operation is in progress, false otherwise.
     */
    public boolean isAutoActive()
    {
        return motorGrabber != null? motorGrabber.isAutoActive(): servoGrabber.isAutoActive();
    }   //isAutoActive

    /**
     * This method cancels the auto-assist operation and to clean up. It is called by the user for canceling the
     * operation.
     */
    public void cancel()
    {
        if (motorGrabber != null)
        {
            motorGrabber.cancel();
        }
        else
        {
            servoGrabber.cancel();
        }
    }   //cancel

    /**
     * This method stops the grabber motor (only applicable for MotorGrabber).
     */
    public void stop(String owner)
    {
        if (motorGrabber != null)
        {
            motorGrabber.stop(owner);
        }
    }   //stop

    /**
     * This method grabs the object manually.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before grabbing the object, 0.0 if no delay.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void intake(String owner, double delay, TrcEvent event)
    {
        if (motorGrabber != null)
        {
            motorGrabber.intake(owner, delay, 0.0, event);
        }
        else
        {
            servoGrabber.close(owner, delay, event);
        }
    }   //intake

    /**
     * This method dumps the object manually.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before dumping the object, 0.0 if no delay.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void dump(String owner, double delay, TrcEvent event)
    {
        if (motorGrabber != null)
        {
            motorGrabber.eject(owner, delay, Grabber.Params.DUMP_TIME, event);
        }
        else
        {
            servoGrabber.open(owner, delay, event);
        }
    }   //dump

    /**
     * This method intakes the object and will stop when the sensor detected the object in possession.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before intaking the object, 0.0 if no delay.
     * @param finishDelay specifies the delay time in seconds to shut off motor, only applicable for motor grabber.
     * @param event specifies the event to signal when the operation is completed.
     * @param timeout specifies the timeout time in seconds, can be zero if no timeout.
     */
    public void autoIntake(String owner, double delay, double finishDelay, TrcEvent event, double timeout)
    {
        if (motorGrabber != null)
        {
            motorGrabber.autoIntake(owner, delay, finishDelay, event, timeout);
        }
        else
        {
            servoGrabber.autoGrab(owner, delay, event, timeout);
        }
    }   //autoIntake

    /**
     * This method intakes the object and will stop when the sensor detected the object in possession.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before intaking the object, 0.0 if no delay.
     * @param finishDelay specifies the delay time in seconds to shut off motor, only applicable for motor grabber.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void autoIntake(String owner, double delay, double finishDelay, TrcEvent event)
    {
        autoIntake(owner, delay, finishDelay, event, 0.0);
    }   //autoIntake

    /**
     * This method dumps the object and will stop when the sensor detected no object in possession.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the time in seconds to delay before dumping the object, 0.0 if no delay.
     * @param finishDelay specifies the delay time in seconds to shut off motor, only applicable for motor grabber.
     * @param event specifies the event to signal when the operation is completed.
     */
    public void autoDump(String owner, double delay, double finishDelay, TrcEvent event)
    {
        if (motorGrabber != null)
        {
            motorGrabber.autoEject(owner, delay, finishDelay, event, 0.0);
        }
        else
        {
            servoGrabber.open(owner, delay, event);
        }
    }   //autoDump

    /**
     * This method reads the analog sensor and returns the distance value.
     *
     * @return analog sensor value, returns zero if no analog sensor.
     */
    public double getSensorDistance()
    {
        if (colorSensor != null)
        {
            return colorSensor.getDistance(DistanceUnit.INCH);
        }
        else
        {
            return 0.0;
        }
    }   //getSensorDistance

    /**
     * This method reads the analog sensor and returns the Hue of the HSV value.
     *
     * @return analog sensor value, returns zero if no analog sensor.
     */
    public double getSensorHue()
    {
        if (colorSensor != null)
        {
            float[] hsvValues = {0.0f, 0.0f, 0.0f};
            NormalizedRGBA normalizedColors = colorSensor.getNormalizedColors();
            Color.RGBToHSV((int) (normalizedColors.red * 255),
                           (int) (normalizedColors.green * 255),
                           (int) (normalizedColors.blue * 255),
                           hsvValues);
            return hsvValues[0];
        }
        else
        {
            return 0.0;
        }
    }   //getSensorHue

    /**
     * This method returns the sample type in the grabber.
     *
     * @return sample type in the grabber, null if no sample.
     */
    public Vision.SampleType getSampleType()
    {
        return sampleType;
    }   //getSampleType

    /**
     * This method is called when the grabber detected a change of color.
     *
     * @param context specifies the trigger callback context.
     */
    private void colorTriggerCallback(Object context)
    {
        double hue = ((TrcTriggerThresholdZones.CallbackContext) context).sensorValue;

        // Red in HSV space crosses the 0/360 cross-over, so the range is actually between <= LOW and >= HIGH.
        if (hue <= Params.RED_THRESHOLD_LOW || hue >= Params.RED_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.RedSample;
        }
        else if (hue >= Params.YELLOW_THRESHOLD_LOW && hue <= Params.YELLOW_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.YellowSample;
        }
        else if (hue >= Params.BLUE_THRESHOLD_LOW && hue <= Params.BLUE_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.BlueSample;
        }
        else if (hue >= Params.BLACK_THRESHOLD_LOW && hue <= Params.BLACK_THRESHOLD_HIGH)
        {
            sampleType = Vision.SampleType.Specimen;
        }
        else
        {
            sampleType = null;
        }

        if (robot.ledIndicator != null)
        {
            robot.ledIndicator.setDetectedSample(sampleType, true);
        }
    }   //colorTriggerCallback

}   //class Grabber
