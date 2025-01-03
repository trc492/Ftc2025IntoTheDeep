// >>> Change to the appropriate package path below.
package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Trc: Simple Auto", group="Autonomous")
// >>> Comment out the Disabled line below.
@Disabled
public class TrcSimpleAuto extends LinearOpMode {
    // >>> Update the following constants appropriately.
    // >>> Update the motor names to their corresponding motor names.
    private static final String LFDRIVE_NAME = "lfDriveMotor";
    private static final String RFDRIVE_NAME = "rfDriveMotor";
    private static final String LBDRIVE_NAME = "lbDriveMotor";
    private static final String RBDRIVE_NAME = "rbDriveMotor";
    // >>> Update their motor directions correspondingly.
    private static final DcMotorSimple.Direction LFDRIVE_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotorSimple.Direction RFDRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotorSimple.Direction LBDRIVE_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotorSimple.Direction RBDRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    // >>> Update to the appropriate auto running time and auto running power.
    private static final double AUTO_RUN_TIME = 2.0;
    private static final double AUTO_RUN_POWER = -0.5;

    @Override
    public void runOpMode() {
        final ElapsedTime runtime = new ElapsedTime();
        final DcMotor lfDrive, rfDrive, lbDrive, rbDrive;

        lfDrive = hardwareMap.get(DcMotor.class, LFDRIVE_NAME);
        rfDrive = hardwareMap.get(DcMotor.class, RFDRIVE_NAME);
        lbDrive = hardwareMap.get(DcMotor.class, LBDRIVE_NAME);
        rbDrive = hardwareMap.get(DcMotor.class, RBDRIVE_NAME);
        lfDrive.setDirection(LFDRIVE_DIRECTION);
        rfDrive.setDirection(RFDRIVE_DIRECTION);
        lbDrive.setDirection(LBDRIVE_DIRECTION);
        rbDrive.setDirection(RBDRIVE_DIRECTION);

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            if (runtime.seconds() < AUTO_RUN_TIME)
            {
                lfDrive.setPower(AUTO_RUN_POWER);
                rfDrive.setPower(AUTO_RUN_POWER);
                lbDrive.setPower(AUTO_RUN_POWER);
                rbDrive.setPower(AUTO_RUN_POWER);
            }
            else
            {
                lfDrive.setPower(0.0);
                rfDrive.setPower(0.0);
                lbDrive.setPower(0.0);
                rbDrive.setPower(0.0);
                break;
            }
        }
    }
}
