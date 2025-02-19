package org.firstinspires.ftc.teamcode.cougears.autonomous.dimby;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.cougears.PresetConstants;
import org.firstinspires.ftc.teamcode.cougears.autonomous.old.AutonomousMethods;

@Autonomous(name="ParkAuton", group="Robot")
public class AutonAlpha extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    AutonomousMethods robot = new AutonomousMethods((DcMotorEx) motorFL, (DcMotorEx) motorFR, (DcMotorEx) motorBL, (DcMotorEx) motorBR);
    // Linear slide motors
    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    private DcMotor armThetaDC;
    private HuskyLens huskyLens;
    private boolean turnedClaw = false;
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

    boolean slow = false;
    private static final int clawRange = 70;
    @Override
    public void runOpMode() {
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
        huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");


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


        // Initialize the drive system variables.

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();

        /*
        Overview:
        This block will strafe until it finds a block in the "sweet zone"
        When it does it decides if it needs to rotate w/ the clawRotationRequired function
        Then it sets foundBlock = true to stop the strafe
         */
        boolean foundBlock = false;
        for (int i = 0; i < 10; i++) {
            if (foundBlock)
                break;
            //TODO: Find a way to strafe. There is nothing for it in auton meathods
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                for (HuskyLens.Block currBlock : blocks)
                {
                    if (currBlock.x < 190 && currBlock.x > 130 && currBlock.y < 150 && currBlock.y > 90)
                    {
                    //Screen is 320x240 and 0,0 = top left, 320,240 = bottom right, 160,120 = center
                    // "Sweet zone rn is 30x30 block in the middle
                        if (clawRotationRequired(currBlock))
                            clawAxis2Servo.setPosition(.5); //TODO: Find the right preset. Rn its just turning 90 deg
                        foundBlock = true;
                    }
                }
            }
        }
        //TODO: Pickup block. Again, idk the presets

    }

    private boolean clawRotationRequired(HuskyLens.Block block) {
        double width = block.width;
        double height = block.height;
        double ratio = Math.min(width, height) / Math.max(width, height);
        double thetaRadians = Math.atan(ratio);
        double thetaDegrees = Math.toDegrees(thetaRadians);
        if (width < height) {
            thetaDegrees = 90 - thetaDegrees;
        }
        thetaDegrees = map(thetaDegrees, 24, 67, 0, 90);
        // Note: This never really worked all that great so I just ended up mapping from 0-90
        if (thetaDegrees > clawRange)
            return true;
        else
            return false;
    }


    public static double map(double value, double originalLow, double originalHigh, double newLow, double newHigh) {
        if(originalLow == originalHigh){
            throw new IllegalArgumentException("Original low cannot equal original high");
        }

        // Calculate the normalized value within the original range.
        double normalizedValue = (value - originalLow) / (originalHigh - originalLow);

        // Map the normalized value to the new range.
        double mappedValue = newLow + normalizedValue * (newHigh - newLow);

        return mappedValue;
    }
}
