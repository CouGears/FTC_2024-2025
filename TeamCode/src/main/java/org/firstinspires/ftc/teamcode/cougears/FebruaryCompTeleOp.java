package org.firstinspires.ftc.teamcode.cougears;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name="February Comp", group="Drive")
public class FebruaryCompTeleOp extends LinearOpMode {
    // Drive motors
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    // Linear slide motors
    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    private DcMotor armThetaDC;

    private Servo clawAxis1Servo;
    private Servo clawAxis2Servo;
    private Servo clawGrabServo;

    private GoBildaPinpointDriver odo;


    // Constants
    private static double MAX_SPEED = 1.0;
    private static double MIN_SPEED = -1.0;
    private static final double SLIDE_POWER = 0.8;
    private static final double SERVO_INCREMENT = 0.02;
    private static final double SERVO_MIN_POS = 0.0;
    private static final double SERVO_MAX_POS = 1.0;

    // position presets

    // init, high drop, high hold, mid, low, custom
    int[] slidePresets = {0, 5400, 5400, 0, 0, 0};
    // init, high drop, high hold, mid, low, custom
    int[] armThetaPresets = {0, 400, 100, 600, 775, 0};
    // init, high drop, high hold, mid, low, custom
    double[] axis1Presets = {0.83, 0.5, 0.5, 0.45, 0.6, 0};
    // init, high drop, high hold, mid, low, custom
    double[] axis2Presets = {0.97, 0.3, 0.3, 0.3, 0.3, 0};
    // open, closed, custom
    double[] clawPresets = {0.4, 0, 0};

    // slide, armtheta, axis1, axis2, claw
    int[] states = new int[5];

    int claw = 1;
    int level = 3;

    boolean upPressed = false;
    boolean downPressed = false;

    boolean slow = false;


    @Override
    public void runOpMode() {
        // Initialize drive motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // Initialize slide motors
        slideLeft = hardwareMap.get(DcMotorEx.class, "viperL");
        slideRight = hardwareMap.get(DcMotorEx.class, "viperR");

        armThetaDC = hardwareMap.get(DcMotor.class, "arm");

        // Initialize servos
        clawAxis1Servo = hardwareMap.get(Servo.class, "axis1");
        clawAxis2Servo = hardwareMap.get(Servo.class, "axis2");
        clawGrabServo = hardwareMap.get(Servo.class, "claw");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set slide motor directions (opposite to maintain sync)
        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(0);
        slideRight.setTargetPosition(0);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(1);
        slideRight.setPower(1);

        armThetaDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armThetaDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armThetaDC.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior for all motors
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servo positions

        states = new int[]{0, 0, 0, 0, 1};
        setStates();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        states = new int[]{3, 3, 0, 0, 1};
        setStates();
        sleep(500);
        states = new int[]{3, 3, 3, 3, 0};
        claw = 0;
        setStates();
        sleep(1500);
//        states = new int[]{3, 3, 3, 3, 0};
//        setStates();
//        sleep(1000);
//        states = new int[]{3, 3, 3, 3, 1};
//        setStates();
//        sleep(1000);
//        states = new int[]{2, 2, 2, 2, 1};
//        setStates();
//        sleep(1000);
//        states = new int[]{1, 1, 1, 1, 1};
//        setStates();
//        sleep(2000);
//        states = new int[]{1, 1, 1, 1, 0};
//        setStates();
//
//        sleep(10000);

        while (opModeIsActive()) {
            // Drive controls
            double drive = gamepad1.left_stick_y; // Forward/back strafe on left stick Y
            double strafe = gamepad1.left_stick_x; // Left/right drive on left stick X
            double rotate = gamepad1.right_stick_x; // Rotation on right stick X

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

            if (gamepad1.x) {
                MAX_SPEED = 1.0;
                MIN_SPEED = -1.0;
            } else if (gamepad1.y) {
                MAX_SPEED = 0.5;
                MIN_SPEED = -0.5;
            }

            if (gamepad2.left_bumper) {
                // open claw
                claw = 0;
                states[4] = claw;
            } else if (gamepad2.right_bumper) {
                // close claw
                claw = 1;
                states[4] = claw;
            }

            // level presets
            if (gamepad2.dpad_up) {
                if (!upPressed) {
                    level--;
                    level = Math.max(1,level);
                    states = new int[]{level, level, level, level, claw};
                    upPressed = true;
                    if (level == 1) {
                        slow = true;
                    } else {
                        slow = false;
                    }
                }
                downPressed = false;
            } else if (gamepad2.dpad_down) {
                if (!downPressed) {
                    level++;
                    level = Math.min(4,level);
                    states = new int[]{level, level, level, level, claw};
                    downPressed = true;
                    if (level == 3) {
                        slow = true;
                    } else {
                        slow = false;
                    }
                }
                upPressed = false;
            } else {
                upPressed = false;
                downPressed = false;
            }

//            if (Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.left_stick_x) > 0.1) {
//                if (gamepad2.left_stick_y > 0) {
//                    axis1Presets[4] = Math.min(1, clawAxis1Servo.getPosition() + (gamepad2.left_stick_y / 10));
//                } else {
//                    axis1Presets[4] = Math.max(0, clawAxis1Servo.getPosition() + (gamepad2.left_stick_y / 10));
//                }
//                if (gamepad2.left_stick_x > 0) {
//                    axis2Presets[4] = Math.min(1, clawAxis2Servo.getPosition() + (gamepad2.left_stick_x / 10));
//                } else {
//                    axis2Presets[4] = Math.max(0, clawAxis2Servo.getPosition() + (gamepad2.left_stick_x / 10));
//                }
//                states[2] = 4;
//                states[3] = 4;
//            }

            if (gamepad2.a) {
                states = new int[]{level, level, level, level, claw};
            }

            if (gamepad2.b) {
                states = new int[]{0, 0, 0, 0, 1};
                slow = false;
            }



            // Apply all motor powers and servo positions
            // Drive motors
            motorFL.setPower(Range.clip(frontLeftPower, MIN_SPEED, MAX_SPEED));
            motorFR.setPower(Range.clip(frontRightPower, MIN_SPEED, MAX_SPEED));
            motorBL.setPower(Range.clip(backLeftPower, MIN_SPEED, MAX_SPEED));
            motorBR.setPower(Range.clip(backRightPower, MIN_SPEED, MAX_SPEED));

            if (slow) {
                setStates(true);
            } else {
                setStates();
            }


            // Telemetry
            telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("States", "%d, %d, %d, %d, %d",
                    states[0], states[1], states[2], states[3], states[4]);
            telemetry.addData("Motor Encoders LR", "FL:%d FR:%d BL:%d BR:%d", motorFL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition(), motorBR.getCurrentPosition());
            telemetry.addData("Slide Encoders LR", "%d, %d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition());
            telemetry.addData("Theta Encoder", "%d", armThetaDC.getCurrentPosition());
            telemetry.addData("Servos AX1, AX2, C", "%f, %f, %f", clawAxis1Servo.getPosition(), clawAxis2Servo.getPosition(), clawGrabServo.getPosition());
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

    private void setStates() {
        for (int i = 0; i < 5; i++) {
            int state = states[i];
            if (state != -1) {
                switch (i) {
                    case 0:
                        slideLeft.setTargetPosition(slidePresets[state]);
                        slideRight.setTargetPosition(slidePresets[state]);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case 1:
                        double power = 0.2;
                        if (state == 1) {
                            power = 0.05;
                        }
                        armThetaDC.setTargetPosition(armThetaPresets[state]);
                        armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armThetaDC.setPower(power);
                        break;
                    case 2:
                        clawAxis1Servo.setPosition(axis1Presets[state]);
                        break;
                    case 3:
                        clawAxis2Servo.setPosition(axis2Presets[state]);
                        break;
                    case 4:
                        clawGrabServo.setPosition(clawPresets[state]);
                        break;
                }
            } else {
                switch (i) {
                    case 0:
                        slideLeft.setTargetPosition(slidePresets[4]);
                        slideRight.setTargetPosition(slidePresets[4]);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case 1:
                        armThetaDC.setTargetPosition(armThetaPresets[4]);
                        armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armThetaDC.setPower(0.2);
                        break;
                    case 2:
                        clawAxis1Servo.setPosition(axis1Presets[4]);
                        break;
                    case 3:
                        clawAxis2Servo.setPosition(axis2Presets[4]);
                        break;
                    case 4:
                        clawGrabServo.setPosition(clawPresets[2]);
                        break;
                }
            }
        }
    }

    private void setStates(boolean slow) {
        for (int i = 0; i < 5; i++) {
            int state = states[i];
            if (state != -1) {
                switch (i) {
                    case 0:
                        slideLeft.setTargetPosition(slidePresets[state]);
                        slideRight.setTargetPosition(slidePresets[state]);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case 1:
                        double power = 0.05;
                        armThetaDC.setTargetPosition(armThetaPresets[state]);
                        armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armThetaDC.setPower(power);
                        break;
                    case 2:
                        clawAxis1Servo.setPosition(axis1Presets[state]);
                        break;
                    case 3:
                        clawAxis2Servo.setPosition(axis2Presets[state]);
                        break;
                    case 4:
                        clawGrabServo.setPosition(clawPresets[state]);
                        break;
                }
            } else {
                switch (i) {
                    case 0:
                        slideLeft.setTargetPosition(slidePresets[4]);
                        slideRight.setTargetPosition(slidePresets[4]);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case 1:
                        armThetaDC.setTargetPosition(armThetaPresets[4]);
                        armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armThetaDC.setPower(0.2);
                        break;
                    case 2:
                        clawAxis1Servo.setPosition(axis1Presets[4]);
                        break;
                    case 3:
                        clawAxis2Servo.setPosition(axis2Presets[4]);
                        break;
                    case 4:
                        clawGrabServo.setPosition(clawPresets[2]);
                        break;
                }
            }
        }
    }
}