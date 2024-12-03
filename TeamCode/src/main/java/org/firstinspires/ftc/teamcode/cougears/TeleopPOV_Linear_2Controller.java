package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/*
    Controller 1: Slide & Bucket
        A: Slide Up
        B: Slide Down
        Y: Switch Control
        X: Speed toggle

        Dpad-Up: n/a
        Dpad-Down: n/a
        Dpad-Left: n/a
        Dpad-Right: n/a

        R-Bumper: n/a
        L-Bumper: n/a
        R-Trigger: Open Bucket
        L-Trigger: Close Bucket

        L-Stick: FWD/BCK & Strafe
        R-Stick: Yaw (Turning)
 */ //Controller 1: Slide & Bucket
/*
    Controller 2: Arm & Claw
        A: Arm Up
        B: Arm Down
        Y: Switch Control
        X: n/a

        Dpad-Up: n/a
        Dpad-Down: n/a
        Dpad-Left: n/a
        Dpad-Right: n/a

        R-Bumper: n/a
        L-Bumper: n/a
        R-Trigger: Open Claw
        L-Trigger: Close Claw

        Note: Controller 2's controls are switched so that the arm is fwd
        L-Stick: FWD/BCK & Strafe
        R-Stick: Yaw (Turning)
 */ //Controller 2: Arm & Claw
@Disabled
@TeleOp(name="Robot: Teleop POV", group="Robot")
public class TeleopPOV_Linear_2Controller extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;
    public DcMotor  arm              = null;
    public DcMotor  slide            = null;
    public Servo    claw             = null;
    public Servo    bucket           = null;

    double clawOffset  = 0;
    double bucketOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.015 ;
    public static final double BUCKET_SPEED = 0.015 ;
    public static final double ARM_UP_POWER    =  0.25 ;
    public static final double ARM_DOWN_POWER  = -0.25 ;
    public static final double SLIDE_UP_POWER   =  0.45 ;
    public static final double SLIDE_DOWN_POWER = -0.45 ;
    public static final double SLOW_SPEED_MULTIPLIER = .6;
    public static final double GENERAL_SPEED_MULTIPLIER = .8;
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
        short activeController = 1; // 1 is bucket/linSlide, 2 is claw/arm. -E
        boolean speedToggle = false; // False = normal speed and True = slow speed

        // Define and Initialize Motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "motorFL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "motorFR");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "motorBL");
        backRightDrive  = hardwareMap.get(DcMotor.class, "motorBR");
        arm             = hardwareMap.get(DcMotor.class, "arm");
        slide           = hardwareMap.get(DcMotor.class, "slide");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

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
            if (activeController == 1) {
                if (speedToggle == true) {
                    drive  = -gamepad1.left_stick_y * SLOW_SPEED_MULTIPLIER;
                    strafe = gamepad1.left_stick_x * SLOW_SPEED_MULTIPLIER;
                    turn   = gamepad1.right_stick_x * SLOW_SPEED_MULTIPLIER;
                } else { // speedToggle == False (AKA normal)
                    drive  = -gamepad1.left_stick_y * GENERAL_SPEED_MULTIPLIER;
                    strafe = gamepad1.left_stick_x * GENERAL_SPEED_MULTIPLIER;
                    turn   = gamepad1.right_stick_x * GENERAL_SPEED_MULTIPLIER;
                }
                telemetry.addData(">", "Controller 1 Active");

            } else if (activeController == 2) { // Just inverted the values. Always slow. -E
                drive  = gamepad2.left_stick_y * SLOW_SPEED_MULTIPLIER;
                strafe = -gamepad2.left_stick_x * SLOW_SPEED_MULTIPLIER;
                turn   = -gamepad2.right_stick_x * SLOW_SPEED_MULTIPLIER;
                telemetry.addData(">", "Controller 2 Active");

            } else { // Should never get to this but safety never hurt anyone. -E
                drive = 0;
                strafe = 0;
                turn = 0;
                telemetry.addData("!ERROR!", "NO CONTROLLER ACTIVE");
            }


            // Mecanum drive calculation
            frontLeft  = drive + turn + strafe;
            backLeft   = drive + turn - strafe;
            frontRight = drive - turn - strafe;
            backRight  = drive - turn + strafe;

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

            if (activeController == 1)
            {
                // Use gamepad buttons to move slide up (A) and down (B)
                if (gamepad1.a)
                    slide.setPower(SLIDE_UP_POWER);
                else if (gamepad1.b)
                    slide.setPower(SLIDE_DOWN_POWER);
                else
                    slide.setPower(0.0);

                // Use gamepad triggers to control the bucket
                if (gamepad1.right_trigger > 0)
                    bucketOffset += BUCKET_SPEED;
                else if (gamepad1.left_trigger > 0)
                    bucketOffset -= BUCKET_SPEED;

                if (gamepad1.x)
                    speedToggle = !speedToggle;

                if (gamepad1.y) // Ability to switch off
                    activeController = 2;
            }
            else if (activeController == 2)
            {
                // Use gamepad buttons to move arm up (Y) and down (A)
                if (gamepad2.a)
                    arm.setPower(ARM_UP_POWER);
                else if (gamepad2.b)
                    arm.setPower(ARM_DOWN_POWER);
                else
                    arm.setPower(0.0);

                // Use gamepad bumpers to open and close the claw
                if (gamepad2.right_trigger > 0)
                    clawOffset += CLAW_SPEED;
                else if (gamepad2.left_trigger > 0)
                    clawOffset -= CLAW_SPEED;

                if (gamepad2.y) // Ability to switch off
                    activeController = 1;
            }

            // Im questioning if the servo calculations should be moved into the if statements. Up to u @josh. -E

            // Move both servos to new position.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            claw.setPosition(MID_SERVO + clawOffset);

            bucketOffset = Range.clip(bucketOffset, -0.5, 0.5);
            bucket.setPosition(MID_SERVO + bucketOffset);

            // Send telemetry message to signify robot running;
            telemetry.addData("Claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("Bucket", "Offset = %.2f", bucketOffset);
            telemetry.addData("Arm",     "%.2f", arm.getPower());
            telemetry.addData("Slide",   "%.2f", slide.getPower());
            telemetry.addData("Front left/right", "%.2f / %.2f", frontLeft, frontRight);
            telemetry.addData("Back left/right", "%.2f / %.2f", backLeft, backRight);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}