package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="HELP ME", group="Drive")
public class HELP extends LinearOpMode {
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
    private static double MAX_SPEED = 1.0;
    private static double MIN_SPEED = -1.0;
    private static final double SLIDE_POWER = 0.8;
    private static final double SERVO_INCREMENT = 0.02;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // Servo position presets
    private static final double[] SERVO_ARM_POS_LIST_A = {0.0, 0.4, 0.5};
    private static final double[] SERVO_ARM_POS_LIST_B = {0.0, 0.4, 0.5};
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

        boolean aPressed = false;
        boolean bPressed = false;
        boolean leftPressed = false;
        boolean rightPressed = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Servos
            bigArmLeft.setPosition(1.0);
            bigArmRight.setPosition(0.0);  // Inverse position for opposite movement
            smallArmLeft.setPosition(lockServoPosition);
            smallArmRight.setPosition(1.0 - lockServoPosition);  // Inverse position for opposite movement

            // Telemetry
            telemetry.addData("Slide Servos", "Position: %.2f", SERVO_ARM_POS_LIST_A[armServoCurrentPosition]);
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