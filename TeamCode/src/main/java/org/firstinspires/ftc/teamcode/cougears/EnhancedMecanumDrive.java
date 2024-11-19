package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="11/14 Competition Drive", group="Drive")
public class EnhancedMecanumDrive extends LinearOpMode {
    // Drive motors
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    // Linear slide motors
    private DcMotor slideLeft;
    private DcMotor slideRight;

    // Servo motors
    private Servo bigArmLeft;
    private Servo bigArmRight;
    private Servo smallArmLeft;
    private Servo smallArmRight;

    // Constants
    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SPEED = -1.0;
    private static final double SLIDE_POWER = 0.8;
    private static final double SERVO_INCREMENT = 0.02;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Servo position presets
    private static final double[] SERVO_ARM_POS_LIST = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    private static final double[] SERVO_LOCK_POS_LIST = {0.0, 0.5, 1.0};


    @Override
    public void runOpMode() {
        // Initialize drive motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // Initialize slide motors
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        // Initialize servos
        bigArmLeft = hardwareMap.get(Servo.class, "bigArmLeft");
        bigArmRight = hardwareMap.get(Servo.class, "bigArmRight");
        smallArmLeft = hardwareMap.get(Servo.class, "smallArmLeft");
        smallArmRight = hardwareMap.get(Servo.class, "smallArmRight");

        // Set motor directions
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set slide motor directions (opposite to maintain sync)
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior for all motors
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servo positions
        double armServoPosition = 0.0;
        double lockServoPosition = 0.0;

        int armServoCurrentPosition = 0;
        int lockServoCurrentPosition = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            double drive = gamepad1.left_stick_y;  // Forward/back strafe on left stick Y
            double strafe = gamepad1.left_stick_x;    // Left/right drive on left stick X
            double rotate = gamepad1.right_stick_x;  // Rotation on right stick X

            // Calculate drive motor powers for strafe-forward configuration
            double frontLeftPower = strafe + drive + rotate;
            double frontRightPower = strafe - drive - rotate;
            double backLeftPower = strafe - drive + rotate;
            double backRightPower = strafe + drive - rotate;

            // Normalize drive motor powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Linear Slides Control (Up/Down arrows)
            double slidePower = 0.0;
            if (gamepad1.dpad_up) {
                slidePower = SLIDE_POWER;
            } else if (gamepad1.dpad_down) {
                slidePower = -SLIDE_POWER;
            }

            // Slide Servo Control (D-PAD LEFT/RIGHT)
            if (gamepad1.dpad_left) {
                if (armServoCurrentPosition <= SERVO_ARM_POS_LIST.length) {
                    armServoCurrentPosition++;
                }
                armServoPosition = SERVO_ARM_POS_LIST[armServoCurrentPosition];
            }
            if (gamepad1.dpad_right) {
                if (armServoCurrentPosition > 0) {
                    armServoCurrentPosition--;
                }
                armServoPosition = SERVO_ARM_POS_LIST[armServoCurrentPosition];
            }

            // Paddle Servo Control (A/B)
            if (gamepad1.a) {
                if (lockServoCurrentPosition <= SERVO_LOCK_POS_LIST.length) {
                    lockServoCurrentPosition++;
                }
                lockServoPosition = SERVO_LOCK_POS_LIST[lockServoCurrentPosition];
            }
            if (gamepad1.b) {
                if (lockServoCurrentPosition > 0) {
                    lockServoCurrentPosition--;
                }
                lockServoPosition = SERVO_LOCK_POS_LIST[lockServoCurrentPosition];
            }

            // Apply all motor powers and servo positions
            // Drive motors
            motorFL.setPower(Range.clip(frontLeftPower, MIN_SPEED, MAX_SPEED));
            motorFR.setPower(Range.clip(frontRightPower, MIN_SPEED, MAX_SPEED));
            motorBL.setPower(Range.clip(backLeftPower, MIN_SPEED, MAX_SPEED));
            motorBR.setPower(Range.clip(backRightPower, MIN_SPEED, MAX_SPEED));

            // Slide motors
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);

            // Servos
            bigArmLeft.setPosition(armServoPosition);
            bigArmRight.setPosition(1.0 - armServoPosition);  // Inverse position for opposite movement
            smallArmLeft.setPosition(lockServoPosition);
            smallArmRight.setPosition(1.0 - lockServoPosition);  // Inverse position for opposite movement

            // Telemetry
            telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Slides", "Power: %.2f", slidePower);
            telemetry.addData("Slide Servos", "Position: %.2f", armServoPosition);
            telemetry.addData("Paddle Servos", "Position: %.2f", lockServoPosition);
            telemetry.update();
        }

        // Stop all motors when OpMode is stopped
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }
}