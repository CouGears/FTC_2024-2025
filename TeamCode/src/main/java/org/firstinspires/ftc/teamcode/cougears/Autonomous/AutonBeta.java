package org.firstinspires.ftc.teamcode.cougears.Autonomous;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.cougears.PresetConstants;

@Autonomous(name="Auto Outline (Beta)", group="Robot")
public class AutonBeta extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
//    AutonomousMethods robot = new AutonomousMethods((DcMotorEx) motorFL, (DcMotorEx) motorFR, (DcMotorEx) motorBL, (DcMotorEx) motorBR);
    // Linear slide motors
    private DcMotor slideLeft;
    private DcMotor slideRight;

    private DcMotor armThetaDC;
    private HuskyLens husky1;
    private HuskyLens husky2;
    private boolean turnedClaw = false;
    private Servo clawAxis1Servo;
    private Servo clawAxis2Servo;
    private Servo clawGrabServo;
    private GoBildaPinpointDriver odo;

    // Constants
    private static double MAX_SPEED = 1.0;
    private static double MIN_SPEED = -1.0;

    // position presets

    // init, high drop, high hold, mid, low, custom
    int[] slidePresets = PresetConstants.slidePresets;
    // init, high drop, high hold, mid, low, custom
    int[] armThetaPresets = PresetConstants.armThetaPresets;
    // init, high drop, high hold, mid, low, custom
    double[] axis1Presets = PresetConstants.axis1Presets;
    // init, high drop, high hold, mid, low, custom
    double[] axis2Presets = PresetConstants.axis2Presets;
    // open, closed, custom
    double[] clawPresets = PresetConstants.clawPresets;

    int[] axis2Positions = PresetConstants.axis2Positions;

    // slide, armtheta, axis1, axis2, claw
    int[] states = new int[5];

    int claw = 1;
    int level = 3;
    int clawLevel = 3;

    boolean upPressed = false;
    boolean downPressed = false;
    boolean leftPressed = false;
    boolean rightPressed = false;

    @Override
    public void runOpMode() {
        // device mapping
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        slideLeft = hardwareMap.get(DcMotor.class, "viperL");
        slideRight = hardwareMap.get(DcMotor.class, "viperR");

        armThetaDC = hardwareMap.get(DcMotor.class, "arm");

        clawAxis1Servo = hardwareMap.get(Servo.class, "axis1");
        clawAxis2Servo = hardwareMap.get(Servo.class, "axis2");
        clawGrabServo = hardwareMap.get(Servo.class, "claw");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        husky1 = hardwareMap.get(HuskyLens.class, "husky1");
        husky1 = hardwareMap.get(HuskyLens.class, "husky2");


        // device setup
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

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        Pose2d beginPose = new Pose2d(-33.0, -63.0, 0.0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        // start pos to field sample 9
        Action action1 = drive.actionBuilder(beginPose)
                .setReversed(true)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-50, -36), Math.toRadians(90))
                .waitSeconds(5)
                //brk
                .setReversed(false)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(5)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(5)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(5)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-56, -25, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(5)
                .setReversed(true)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(225))
                .build();
        // field sample 9 to bucket
//        Action action2 = drive.actionBuilder(new Pose2d)
        Action action2 = drive.actionBuilder(new Pose2d(-48.0, -38.0, Math.toRadians(0)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(135)), Math.toRadians(135))
                .build();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(action1);


    }

    // slides, arm, axis1, axis2, claw
    private void setStates(int slides, int arm, double axis1, double axis2, double claw) {

            slideLeft.setTargetPosition(slides);
            slideRight.setTargetPosition(slides);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double power = 0.2;
            armThetaDC.setTargetPosition(arm);
            armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armThetaDC.setPower(power);

            clawAxis1Servo.setPosition(axis1);

            clawAxis2Servo.setPosition(axis2);

            clawGrabServo.setPosition(claw);
    }
}
