package org.firstinspires.ftc.teamcode.cougears.autonomous.rr.autons.red.bucket;

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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.drive.TAB;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.hardware.SampleActions;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.*;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.sequence.ROHardcoded.*;

import static org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.sequence.RBHardcoded.AutonRB1;



@Autonomous(name="AutonRB1", group="Robot")
public class AutonRB1 extends LinearOpMode {

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
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");


        // device setup
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d beginPose = TAB.RBIpose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Viper viper  = new Viper(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Axis1 axis1 = new Axis1(hardwareMap);
        Axis2 axis2 = new Axis2(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(AutonRB1(drive, viper, arm, claw, axis1, axis2));
    }
}
