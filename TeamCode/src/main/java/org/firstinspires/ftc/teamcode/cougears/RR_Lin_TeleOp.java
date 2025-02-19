//package org.firstinspires.ftc.teamcode.cougears;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
///*
//    ROBOT CONTROLS:
//    X: LinSlide Up
//    B: LinSlide Down
//    Y: Arm Up
//    A: Arm Down
//
//    Dpad-Up:
//    Dpad-Down:
//    Dpad-Left:
//    Dpad-Right:
//
//    R-Bumper: Open Claw
//    L-Bumper: Close Claw
//    R-Trigger: Open Bucket
//    L-Trigger: Close Bucket
//
//    L-Stick: FWD/BCK & Strafe
//    R-Stick: Yaw (Turning)
// */
//
//@TeleOp(name="RR TeleOp", group="Robot")
//public class RR_Lin_TeleOp extends LinearOpMode {
//    // Drive system
//    private MecanumDrive drive;
//
//    // Additional mechanisms
//    private DcMotorEx arm;
//    private DcMotorEx slide;
//    private Servo claw;
//    private Servo bucket;
//
//    // Servo positions and offsets
//    double clawOffset = 0;
//    double bucketOffset = 0;
//    public static final double MID_SERVO = 0.5;
//    public static final double CLAW_SPEED = 0.02;
//    public static final double BUCKET_SPEED = 0.02;
//    public static final double ARM_POWER = 0.5;
//    public static final double SLIDE_POWER = 0.5;
//
//    @Override
//    public void runOpMode() {
//        // Initialize drive system
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        // Initialize additional mechanisms
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        slide = hardwareMap.get(DcMotorEx.class, "slide");
//        claw = hardwareMap.get(Servo.class, "claw");
//        bucket = hardwareMap.get(Servo.class, "bucket");
//
//        // Configure arm and slide motors
//        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        // Initialize servo positions
//        claw.setPosition(MID_SERVO);
//        bucket.setPosition(MID_SERVO);
//
//        telemetry.addData(">", "Robot Ready.  Press START.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Update robot position estimate
//            drive.updatePoseEstimate();
//
//            // Get drive inputs
//            double drive_x = -gamepad1.left_stick_y; // Forward/back
//            double drive_y = -gamepad1.left_stick_x; // Strafe
//            double turn = -gamepad1.right_stick_x;   // Turn
//
//            // Create velocity vector and set drive powers
//            Vector2d driveVector = new Vector2d(drive_x, drive_y);
//            drive.setDrivePowers(new PoseVelocity2d(driveVector, turn));
//
//            // Arm control
//            if (gamepad1.y) {
//                arm.setPower(ARM_POWER);
//            } else if (gamepad1.a) {
//                arm.setPower(-ARM_POWER);
//            } else {
//                arm.setPower(0);
//            }
//
//            // Slide control
//            if (gamepad1.x) {
//                slide.setPower(SLIDE_POWER);
//            } else if (gamepad1.b) {
//                slide.setPower(-SLIDE_POWER);
//            } else {
//                slide.setPower(0);
//            }
//
//            // Claw control
//            if (gamepad1.right_bumper) {
//                clawOffset += CLAW_SPEED;
//            } else if (gamepad1.left_bumper) {
//                clawOffset -= CLAW_SPEED;
//            }
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            claw.setPosition(MID_SERVO + clawOffset);
//
//            // Bucket control
//            if (gamepad1.right_trigger > 0) {
//                bucketOffset += BUCKET_SPEED;
//            } else if (gamepad1.left_trigger > 0) {
//                bucketOffset -= BUCKET_SPEED;
//            }
//            bucketOffset = Range.clip(bucketOffset, -0.5, 0.5);
//            bucket.setPosition(MID_SERVO + bucketOffset);
//
//            // Telemetry
//            Pose2d poseEstimate = drive.pose;
//            telemetry.addData("x", poseEstimate.position.x);
//            telemetry.addData("y", poseEstimate.position.y);
//            telemetry.addData("heading", Math.toDegrees(poseEstimate.heading.toDouble()));
//            telemetry.addData("claw", "Offset = %.2f", clawOffset);
//            telemetry.addData("bucket", "Offset = %.2f", bucketOffset);
//            telemetry.addData("arm", "Position = %d", arm.getCurrentPosition());
//            telemetry.addData("slide", "Position = %d", slide.getCurrentPosition());
//            telemetry.update();
//
//            sleep(50);
//        }
//    }
//}