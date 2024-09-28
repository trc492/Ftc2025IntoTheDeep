
package teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import ftclib.subsystem.FtcServoGrabber;
import teamcode.RobotParams;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a Grabber Subsystem.
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
        if (RobotParams.Grabber.USE_REV_2M_SENSOR)
        {
            rev2mSensor = FtcOpMode.getInstance().hardwareMap.get(
                    Rev2mDistanceSensor.class, RobotParams.Grabber.REV_2M_SENSOR_NAME);
        }
        else
        {
            rev2mSensor = null;
        }

        FtcServoGrabber.Params grabberParams = new FtcServoGrabber.Params()
                .setPrimaryServo(RobotParams.Grabber.PRIMARY_SERVO_NAME, RobotParams.Grabber.PRIMARY_SERVO_INVERTED)
                .setFollowerServo(RobotParams.Grabber.FOLLOWER_SERVO_NAME, RobotParams.Grabber.FOLLOWER_SERVO_INVERTED)
                .setOpenCloseParams(RobotParams.Grabber.OPEN_POS, RobotParams.Grabber.OPEN_TIME,
                        RobotParams.Grabber.CLOSE_POS, RobotParams.Grabber.CLOSE_TIME);

        if (rev2mSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                    this::getSensorData, RobotParams.Grabber.ANALOG_TRIGGER_INVERTED,
                    RobotParams.Grabber.SENSOR_TRIGGER_THRESHOLD, RobotParams.Grabber.HAS_OBJECT_THRESHOLD,
                    null);
        }
        else if (RobotParams.Grabber.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(
                    RobotParams.Grabber.REV_2M_SENSOR_NAME, RobotParams.Grabber.DIGITAL_TRIGGER_INVERTED, null);
        }

        grabber = new FtcServoGrabber(RobotParams.Grabber.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }

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
    }

}   //class Grabber