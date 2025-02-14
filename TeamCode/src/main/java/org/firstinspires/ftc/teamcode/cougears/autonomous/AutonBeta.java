package org.firstinspires.ftc.teamcode.cougears.autonomous;/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.cougears.autonomous.RRHardware.*;


@Autonomous(name="Auton Beta", group="Robot")
public class AutonBeta extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    private GoBildaPinpointDriver odo;

    // slide, armtheta, axis1, axis2, claw
    int[] states = new int[5];

    int claw = 1;
    int level = 3;
    int clawLevel = 3;

    boolean upPressed = false;
    boolean downPressed = false;
    boolean leftPressed = false;
    boolean rightPressed = false;

    public Action actionwait(PinpointDrive drive, double time) {
        return drive.actionBuilder(new Pose2d(0, 0, 0))
                .waitSeconds(time)
                .build();
    }

    @Override
    public void runOpMode() {
        // device mapping
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // device setup
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d beginPose = new Pose2d(-33.0, -63.0, 0.0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Viper viper  = new Viper(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Axis1 axis1 = new Axis1(hardwareMap);
        Axis2 axis2 = new Axis2(hardwareMap);

        // start pos to field sample 9
        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                .setReversed(true)
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(-48.0, -40.0), Math.toRadians(90));
        // field sample 9 to bucket
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .setReversed(false)
                .setTangent(Math.toRadians(270))
                .strafeTo(new Vector2d(-48.0, -48.0))
                .turnTo(Math.toRadians(135))
                .endTrajectory();
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-50, -36, Math.toRadians(45)), Math.toRadians(135));

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-56, -47, Math.toRadians(135)), Math.toRadians(225));

        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-65, -24, Math.toRadians(90)), Math.toRadians(180));

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-56, -47, Math.toRadians(135)), Math.toRadians(225));

        SequentialAction grabSample = new SequentialAction(
                claw.openClaw(),
                axis1.axis1ToLevel(4),
                arm.armToLevelPatientCustomSpeed(4, 0.15),
                claw.closeClaw(),
                actionwait(drive, 0.3),
                axis1.axis1ToLevel(3),
                arm.armUpToLevelPatient(3)
        );

        SequentialAction dropSampleInHighBucket = new SequentialAction(
                arm.armUpToLevelImpatient(2),
                axis1.axis1ToLevel(2),
                viper.viperToLevelPatient(2),
                axis1.axis1ToLevel(1),
                arm.armUpToLevelPatient(1),
                actionwait(drive, 0.5),
                claw.openClaw(),
                actionwait(drive, 0.5),
                arm.armDownToLevelImpatient(2),
                viper.viperToPatient(3),
                arm.armDownToLevelImpatient(3),
                axis1.axis1ToLevel(3)
        );

        Action tabsig = drive.actionBuilder(beginPose)
                .turn(Math.toRadians(10800))
                .build();

        SequentialAction action1 = new SequentialAction(
                tab1.build(),
                grabSample,
                tab2.build(),
                dropSampleInHighBucket
        );



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();

//        Actions.runBlocking(action1);
        Actions.runBlocking(tabsig);
    }

    // slides, arm, axis1, axis2, claw
//    private void setStates(int slides, int arm, double axis1, double axis2, double claw) {
//
//            slideLeft.setTargetPosition(slides);
//            slideRight.setTargetPosition(slides);
//            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            double power = 0.2;
//            armThetaDC.setTargetPosition(arm);
//            armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armThetaDC.setPower(power);
//
//            clawAxis1Servo.setPosition(axis1);
//
//            clawAxis2Servo.setPosition(axis2);
//
//            clawGrabServo.setPosition(claw);
//    }
//
//    private void setStates(int slides, int arm, double axis1, double axis2, double claw, double pwr) {
//
//        slideLeft.setTargetPosition(slides);
//        slideRight.setTargetPosition(slides);
//        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        double power = 0.2;
//        armThetaDC.setTargetPosition(arm);
//        armThetaDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armThetaDC.setPower(power);
//
//        clawAxis1Servo.setPosition(axis1);
//
//        clawAxis2Servo.setPosition(axis2);
//
//        clawGrabServo.setPosition(claw);
//    }
}
