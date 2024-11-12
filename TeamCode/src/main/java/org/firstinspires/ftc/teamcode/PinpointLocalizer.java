package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        // Offsets from tracking center (in mm)
        public double xPodOffset = -84.0; // X pod offset (left is positive)
        public double yPodOffset = -168.0; // Y pod offset (forward is positive)
    }

    public static Params PARAMS = new Params();

    private final GoBildaPinpointDriver pinpoint;
    private boolean initialized = false;
    private double lastX, lastY, lastHeading;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        // Initialize the Pinpoint driver
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure for 4-bar pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set the pod offsets
        pinpoint.setOffsets(PARAMS.xPodOffset, PARAMS.yPodOffset);

        // Set encoder directions - adjust these if needed based on your pod mounting
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset position and calibrate IMU
        pinpoint.resetPosAndIMU();
    }

    @Override
    public Twist2dDual<Time> update() {
        // Update Pinpoint readings
        pinpoint.update();

        // Get current position from Pinpoint (in mm)
        double currentX = pinpoint.getPosX();
        double currentY = pinpoint.getPosY();
        double currentHeading = pinpoint.getHeading();

        // Get velocities from Pinpoint (in mm/s and rad/s)
        double xVel = pinpoint.getVelX();
        double yVel = pinpoint.getVelY();
        double headingVel = pinpoint.getHeadingVelocity();

        if (!initialized) {
            initialized = true;
            lastX = currentX;
            lastY = currentY;
            lastHeading = currentHeading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Calculate position deltas in millimeters
        double dx = currentX - lastX;
        double dy = currentY - lastY;
        double dh = currentHeading - lastHeading;

        // Update last positions
        lastX = currentX;
        lastY = currentY;
        lastHeading = currentHeading;

        // Create the twist with both position and velocity information
        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                dx / 25.4, // Convert mm to inches for RoadRunner
                                xVel / 25.4 // Convert mm/s to in/s
                        }),
                        new DualNum<Time>(new double[] {
                                dy / 25.4, // Convert mm to inches for RoadRunner
                                yVel / 25.4 // Convert mm/s to in/s
                        })
                ),
                new DualNum<>(new double[] {
                        dh, // Heading change in radians
                        headingVel // Angular velocity in rad/s
                })
        );
    }
}