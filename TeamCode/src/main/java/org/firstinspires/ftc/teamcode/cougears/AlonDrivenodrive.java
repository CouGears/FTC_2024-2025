package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Alon Drive no drive", group="Drive")
public class AlonDrivenodrive extends LinearOpMode {
    // Drive motors
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    // Linear slide motors
    private DcMotor slide;

    // Servo motors
    private Servo servo1;
    private CRServo servo2;

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
//        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
//        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
//        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
//        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // Initialize slide motors
        slide = hardwareMap.get(DcMotor.class, "slide");

        // Initialize servos
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        // Set motor directions
//        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set slide motor directions (opposite to maintain sync)
        slide.setDirection(DcMotorSimple.Direction.FORWARD);


//        // Set zero power behavior for all motors
//        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize servo positions
        double servo1pos = 0.0;
        double servo2power = 0.0;



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

            // Servo1 (D-PAD LEFT/RIGHT)
            if (gamepad1.dpad_left) {
                servo1pos  += SERVO_INCREMENT;
            }
            if (gamepad1.dpad_right) {
                servo1pos  -= SERVO_INCREMENT;
            }

            // Servo2 (A/B)
            if (gamepad1.a) {

                servo2power = 1;
            }
            else if (gamepad1.b) {
                servo2power = -1;
            }
            else {
                servo2power = 0;
            }

            // Apply all motor powers and servo positions
            // Drive motors
//            motorFL.setPower(Range.clip(frontLeftPower, MIN_SPEED, MAX_SPEED));
//            motorFR.setPower(Range.clip(frontRightPower, MIN_SPEED, MAX_SPEED));
//            motorBL.setPower(Range.clip(backLeftPower, MIN_SPEED, MAX_SPEED));
//            motorBR.setPower(Range.clip(backRightPower, MIN_SPEED, MAX_SPEED));

            // Slide motors
            slide.setPower(slidePower);

            // Servos
            servo1.setPosition(servo1pos);
            servo2.setPower(servo2power);

            // Telemetry
            telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Slide", "Power: %.2f", slidePower);
            telemetry.addData("Servo1", "Position: %.2f", servo1pos);
            telemetry.addData("Servo2", "Position: %.2f", servo2power);
            telemetry.update();
        }

        // Stop all motors when OpMode is stopped
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
        slide.setPower(0);
    }
}