package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/*
    ROBOT CONTROLS:
    X: LinSlide Up
    B: LinSlide Down
    Y: Arm Up
    A: Arm Down

    Dpad-Up:
    Dpad-Down:
    Dpad-Left:
    Dpad-Right:

    R-Bumper: Open Claw
    L-Bumper: Close Claw
    R-Trigger: Open Bucket
    L-Trigger: Close Bucket

    L-Stick: FWD/BCK & Strafe
    R-Stick: Yaw (Turning)
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class RobotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx frontLeftDrive   = null;
    public DcMotorEx frontRightDrive  = null;
    public DcMotorEx backLeftDrive    = null;
    public DcMotorEx backRightDrive   = null;
    public DcMotorEx arm              = null;
    public DcMotorEx slide            = null;
    public Servo    claw             = null;
    public Servo    bucket           = null;

    double clawOffset  = 0;
    double bucketOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;
    public static final double BUCKET_SPEED = 0.02 ;
    public static final double ARM_POWER    =  0.5 ;
    public static final double SLIDE_POWER   =  0.5 ;

    @Override
    public void runOpMode() {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double drive;
        double turn;
        double strafe;
        double max;

        // Define and Initialize Motors
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "motorFL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "motorFR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "motorBL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "motorBR");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        // Set zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set up encoders for arm and slide
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        claw   = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        claw.setPosition(MID_SERVO);
        bucket.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, backward, and strafe
            // Right stick to turn left and right.
            drive  = -gamepad1.left_stick_x;  // Negative because the gamepad is inverted
            strafe = gamepad1.left_stick_y;
            turn   = gamepad1.right_stick_x;

            // Corrected Mecanum drive calculation
            frontLeft  = drive - turn - strafe;
            backLeft   = drive - turn + strafe;
            frontRight = drive + turn + strafe;
            backRight  = drive + turn - strafe;

            // Normalize the values so none exceed +/- 1.0
            max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                    Math.max(Math.abs(frontRight), Math.abs(backRight))));
            if (max > 1.0) {
                frontLeft  /= max;
                backLeft   /= max;
                frontRight /= max;
                backRight  /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeft);
            backLeftDrive.setPower(backLeft);
            frontRightDrive.setPower(frontRight);
            backRightDrive.setPower(backRight);

            // Use gamepad bumpers to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move claw servo to new position.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            claw.setPosition(MID_SERVO + clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.y)
                arm.setPower(ARM_POWER);
            else if (gamepad1.a)
                arm.setPower(-ARM_POWER);
            else
                arm.setPower(0);

            // Use gamepad buttons to move slide up (X) and down (B)
            if (gamepad1.x)
                slide.setPower(SLIDE_POWER);
            else if (gamepad1.b)
                slide.setPower(-SLIDE_POWER);
            else
                slide.setPower(0);

            // Use gamepad triggers to control the bucket
            if (gamepad1.right_trigger > 0)
                bucketOffset += BUCKET_SPEED;
            else if (gamepad1.left_trigger > 0)
                bucketOffset -= BUCKET_SPEED;

            bucketOffset = Range.clip(bucketOffset, -0.5, 0.5);
            bucket.setPosition(MID_SERVO + bucketOffset);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("bucket", "Offset = %.2f", bucketOffset);
            telemetry.addData("arm", "Position = %d", arm.getCurrentPosition());
            telemetry.addData("slide", "Position = %d", slide.getCurrentPosition());
            telemetry.addData("front left/right", "%.2f / %.2f", frontLeft, frontRight);
            telemetry.addData("back left/right", "%.2f / %.2f", backLeft, backRight);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}