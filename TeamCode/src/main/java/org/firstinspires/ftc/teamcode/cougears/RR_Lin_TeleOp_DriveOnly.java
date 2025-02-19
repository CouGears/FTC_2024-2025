//package org.firstinspires.ftc.teamcode.cougears;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.PinpointLocalizer;
//
//
///*
// * Drive Controls:
// * Left Stick Y - Forward/Backward
// * Left Stick X - Strafe Left/Right
// * Right Stick X - Turn Left/Right
// *
// * A - Reset pose to origin (0,0,0)
// * B - Reset IMU (recalibrate when stationary)
// * Y - Toggle drive mode (robot centric vs. field centric)
// */
//@TeleOp(name="RR Lin TeleOp Drive Only", group="Drive")
//public class RR_Lin_TeleOp_DriveOnly extends LinearOpMode {
//    private MecanumDrive drive;
//    private GoBildaPinpointDriver pinpoint;
//    private boolean fieldCentric = false;
//    private double speedMultiplier = 1.0;
//
//    @Override
//    public void runOpMode() {
//        // Initialize drive
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//
//        // Get Pinpoint driver directly
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Mode", fieldCentric ? "Field Centric" : "Robot Centric");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Update robot position
//            drive.updatePoseEstimate();
//            Pose2d currentPose = drive.pose;
//
//            // Get drive inputs (reversed Y since joystick up is negative)
//            double driveX = -gamepad1.left_stick_y; // Forward
//            double driveY = -gamepad1.left_stick_x; // Strafe
//            double turn = -gamepad1.right_stick_x;  // Turn
//
//            // Speed control with triggers
//            speedMultiplier = 1.0 - (0.7 * gamepad1.right_trigger); // Right trigger slows down
//
//            // Handle field centric driving
//            if (fieldCentric) {
//                // Rotate the drive vector by the robot's heading
//                double heading = currentPose.heading.toDouble();
//                double rotX = driveX * Math.cos(-heading) - driveY * Math.sin(-heading);
//                double rotY = driveX * Math.sin(-heading) + driveY * Math.cos(-heading);
//                driveX = rotX;
//                driveY = rotY;
//            }
//
//            // Apply speed multiplier
//            driveX *= speedMultiplier;
//            driveY *= speedMultiplier;
//            turn *= speedMultiplier;
//
//            // Set drive powers using RoadRunner's kinematics
//            Vector2d driveVector = new Vector2d(driveX, driveY);
//            drive.setDrivePowers(new PoseVelocity2d(driveVector, turn));
//
//            // Handle additional controls
//            if (gamepad1.a) {
//                // Reset pose to origin and recalibrate IMU
//                pinpoint.resetPosAndIMU();
//                drive.pose = new Pose2d(0, 0, 0);
//            }
//
//            if (gamepad1.b) {
//                // Recalibrate IMU - robot must be stationary
//                pinpoint.recalibrateIMU();
//            }
//
//            if (gamepad1.y && !wasYPressed) {
//                // Toggle field centric mode
//                fieldCentric = !fieldCentric;
//            }
//            wasYPressed = gamepad1.y;
//
//            // Telemetry
//            telemetry.addData("Mode", fieldCentric ? "Field Centric" : "Robot Centric");
//            telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);
//
//            telemetry.addData("Position",
//                    String.format("X: %.2f, Y: %.2f, Heading: %.1f°",
//                            currentPose.position.x,
//                            currentPose.position.y,
//                            Math.toDegrees(currentPose.heading.toDouble())));
//
//            // Get pinpoint status
//            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
//            telemetry.addData("Update Frequency", "%.1f Hz", pinpoint.getFrequency());
//
//            // Movement data
//            telemetry.addData("Drive Input",
//                    String.format("Drive: (%.2f, %.2f), Turn: %.2f",
//                            driveX, driveY, turn));
//            telemetry.update();
//        }
//    }
//
//    private boolean wasYPressed = false;
//}