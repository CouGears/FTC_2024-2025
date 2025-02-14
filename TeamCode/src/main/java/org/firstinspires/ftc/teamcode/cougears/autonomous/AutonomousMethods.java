package org.firstinspires.ftc.teamcode.cougears.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonomousMethods {

    // Robot hardware
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx arm, slide;
    private Servo claw, bucket;

    // Constants
    public static final double DRIVE_POWER = 0.5;
    private static final double TURN_POWER = 0.4;
    private static final double ARM_POWER = 0.5;
    private static final double SLIDE_POWER = 0.5;
    private static final double COUNTS_PER_INCH = 100; // Adjust based on your robot's configuration
    private static final double MID_SERVO = 0.5;

    // Timer for movement durations
    private ElapsedTime runtime = new ElapsedTime();

    public AutonomousMethods(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br
) {
        this.frontLeftDrive = fl;
        this.frontRightDrive = fr;
        this.backLeftDrive = bl;
        this.backRightDrive = br;
//        this.arm = arm;
//        this.slide = slide;
//        this.claw = claw;
//        this.bucket = bucket;

        // Set zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set up encoders
//        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double inches) {
        int targetPosition = (int)(inches * COUNTS_PER_INCH);
        setDriveTargetPosition(targetPosition);
        setDrivePower(DRIVE_POWER);
        while (isDriveBusy()) {
            // Wait for drive motors to reach the target position
        }
        stopDrive();
    }

    public void driveBackward(double inches) {
        driveForward(-inches);
    }

    public void turnRight(double degrees) {
        // Implement turning logic based on your robot's configuration
        // This is a simple time-based approach; consider using gyro for more accuracy
        runtime.reset();
        setTurnPower(TURN_POWER);
        while (runtime.seconds() < degrees / 100) {
            // Wait for turn to complete
        }
        stopDrive();
    }

    public void turnLeft(double degrees) {
        turnRight(-degrees);
    }

//    public void moveArm(double power, long durationMs) {
//        arm.setPower(power);
//        sleep(durationMs);
//        arm.setPower(0);
//    }

//    public void moveSlide(double power, long durationMs) {
//        slide.setPower(power);
//        sleep(durationMs);
//        slide.setPower(0);
//    }

//    public void setClaw(double position) {
//        claw.setPosition(Range.clip(position, 0, 1));
//    }

//    public void setBucket(double position) {
//        bucket.setPosition(Range.clip(position, 0, 1));
//    }

//    public void interactWithBucket() {
//        // Example interaction: Lower arm, open claw, raise arm
//        moveArm(-ARM_POWER, 1000); // Lower arm
//        setClaw(0.7); // Open claw
//        sleep(500);
//        moveArm(ARM_POWER, 1000); // Raise arm
//    }

    private void setDriveTargetPosition(int position) {
        frontLeftDrive.setTargetPosition(position);
        frontRightDrive.setTargetPosition(position);
        backLeftDrive.setTargetPosition(position);
        backRightDrive.setTargetPosition(position);

        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setDrivePower(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void setTurnPower(double power) {
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    private boolean isDriveBusy() {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                backLeftDrive.isBusy() && backRightDrive.isBusy();
    }

    private void stopDrive() {
        setDrivePower(0);
        frontLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}