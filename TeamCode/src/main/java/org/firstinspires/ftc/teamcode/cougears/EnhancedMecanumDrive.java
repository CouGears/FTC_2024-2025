package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Enhanced Mecanum Drive", group="Drive")
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
        double slideServoPosition = 0.5;
        double paddleServoPosition = 0.5;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Calculate drive motor powers
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

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

            // Slide Servo Control (Triggers)
            if (gamepad1.left_trigger > 0) {
                slideServoPosition = Math.max(slideServoPosition - SERVO_INCREMENT, SERVO_MIN_POS);
            }
            if (gamepad1.right_trigger > 0) {
                slideServoPosition = Math.min(slideServoPosition + SERVO_INCREMENT, SERVO_MAX_POS);
            }

            // Paddle Servo Control (Bumpers/Paddles)
            if (gamepad1.a) {
                paddleServoPosition = Math.max(paddleServoPosition - SERVO_INCREMENT, SERVO_MIN_POS);
            }
            if (gamepad1.b) {
                paddleServoPosition = Math.min(paddleServoPosition + SERVO_INCREMENT, SERVO_MAX_POS);
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
            bigArmLeft.setPosition(slideServoPosition);
            bigArmRight.setPosition(1.0 - slideServoPosition);  // Inverse position for opposite movement
            smallArmLeft.setPosition(paddleServoPosition);
            smallArmRight.setPosition(1.0 - paddleServoPosition);  // Inverse position for opposite movement

            // Telemetry
            telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Slides", "Power: %.2f", slidePower);
            telemetry.addData("Slide Servos", "Position: %.2f", slideServoPosition);
            telemetry.addData("Paddle Servos", "Position: %.2f", paddleServoPosition);
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