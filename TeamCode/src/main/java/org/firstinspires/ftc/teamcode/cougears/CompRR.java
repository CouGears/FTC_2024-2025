package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Disabled
@TeleOp(name="11/14 Competition Drive with Odometry", group="Drive")
public class CompRR extends LinearOpMode {
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

    // Odometry computer
    private GoBildaPinpointDriver odometry;

    // Constants
    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SPEED = -1.0;
    private static final double SLIDE_POWER = 0.8;
    private static final double SERVO_INCREMENT = 0.02;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Slide encoder positions
    private static final int SLIDE_HOME = 0;
    private static final int SLIDE_LOW = 1000;
    private static final int SLIDE_MEDIUM = 2000;
    private static final int SLIDE_HIGH = 3000;

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

        // Initialize odometry
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.setOffsets(-84.0, -168.0); // Adjust these values based on your robot's measurements
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();

        // Set motor directions
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set slide motor directions and modes
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure slide motors for position control
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set zero power behavior for drive motors
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servo positions
        double armServoPosition = 0.0;
        double lockServoPosition = 0.0;
        int armServoCurrentPosition = 0;
        int lockServoCurrentPosition = 0;
        int targetSlidePosition = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            odometry.update();

            // Drive controls with field-centric calculations
            double drive = -gamepad1.left_stick_y;  // Forward/back
            double strafe = gamepad1.left_stick_x;  // Left/right
            double rotate = gamepad1.right_stick_x; // Rotation

            // Get robot's current heading from odometry
            double heading = odometry.getHeading();

            // Field-centric drive calculations
            double rotX = drive * Math.cos(heading) - strafe * Math.sin(heading);
            double rotY = drive * Math.sin(heading) + strafe * Math.cos(heading);

            // Calculate motor powers using field-centric values
            double frontLeftPower = rotY + rotX + rotate;
            double frontRightPower = rotY - rotX - rotate;
            double backLeftPower = rotY - rotX + rotate;
            double backRightPower = rotY + rotX - rotate;

            // Normalize motor powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Linear Slides Control with encoder positions
            if (gamepad1.y) {
                targetSlidePosition = SLIDE_HIGH;
            } else if (gamepad1.x) {
                targetSlidePosition = SLIDE_MEDIUM;
            } else if (gamepad1.a) {
                targetSlidePosition = SLIDE_LOW;
            } else if (gamepad1.b) {
                targetSlidePosition = SLIDE_HOME;
            }

            // Manual slide adjustment
            if (gamepad1.dpad_up) {
                targetSlidePosition += 50;
            } else if (gamepad1.dpad_down) {
                targetSlidePosition -= 50;
            }

            // Ensure slide position is within bounds
            targetSlidePosition = Range.clip(targetSlidePosition, SLIDE_HOME, SLIDE_HIGH);

            // Set slide motor positions
            slideLeft.setTargetPosition(targetSlidePosition);
            slideRight.setTargetPosition(targetSlidePosition);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(SLIDE_POWER);
            slideRight.setPower(SLIDE_POWER);

            // Servo Control (Left/Right bumpers)
            if (gamepad1.left_bumper) {
                if (armServoCurrentPosition < SERVO_ARM_POS_LIST.length - 1) {
                    armServoCurrentPosition++;
                }
                armServoPosition = SERVO_ARM_POS_LIST[armServoCurrentPosition];
            }
            if (gamepad1.right_bumper) {
                if (armServoCurrentPosition > 0) {
                    armServoCurrentPosition--;
                }
                armServoPosition = SERVO_ARM_POS_LIST[armServoCurrentPosition];
            }

            // Apply motor powers
            motorFL.setPower(Range.clip(frontLeftPower, MIN_SPEED, MAX_SPEED));
            motorFR.setPower(Range.clip(frontRightPower, MIN_SPEED, MAX_SPEED));
            motorBL.setPower(Range.clip(backLeftPower, MIN_SPEED, MAX_SPEED));
            motorBR.setPower(Range.clip(backRightPower, MIN_SPEED, MAX_SPEED));

            // Apply servo positions
            bigArmLeft.setPosition(armServoPosition);
            bigArmRight.setPosition(1.0 - armServoPosition);
            smallArmLeft.setPosition(lockServoPosition);
            smallArmRight.setPosition(1.0 - lockServoPosition);

            // Telemetry
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f, H: %.2f",
                    odometry.getPosX(), odometry.getPosY(),
                    Math.toDegrees(odometry.getHeading()));
            telemetry.addData("Slide Position", "Target: %d, Current: %d",
                    targetSlidePosition, slideLeft.getCurrentPosition());
            telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Servos", "Arm: %.2f, Lock: %.2f",
                    armServoPosition, lockServoPosition);
            telemetry.addData("Status", odometry.getDeviceStatus());
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