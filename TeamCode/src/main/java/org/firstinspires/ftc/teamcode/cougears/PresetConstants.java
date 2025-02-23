package org.firstinspires.ftc.teamcode.cougears;


import com.acmerobotics.dashboard.config.Config;

@Config
public class PresetConstants {
    // init, high drop, high hold, hang, low, specimen
    public static int[] slidePresets = {0, 4000, 4000, 2000, 0, 0};
    // init, high drop, high hold, mid, low, specimen
    public static int[] armThetaPresets = {0, 200, 100, 600, 700, 820};
    // init, high drop, high hold, mid, low, specimen
    public static double[] axis1Presets = {0.38, 0.5, 0.6, 0.57, 0.63, 1};
    public static int[] axis2Positions = {2, 1, 0, 3, 4, 2};

    // center, half left, full left, half right, full right, specimen
    public static double[] axis2Presets = {0.43, 0.595, 0.76, 0.265, 0.1, 0.43}; // good
    // open, closed
    public static double[] clawPresets = {0.35, 0.26}; // good
}
