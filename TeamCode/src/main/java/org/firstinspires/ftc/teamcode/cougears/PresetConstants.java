package org.firstinspires.ftc.teamcode.cougears;


import com.acmerobotics.dashboard.config.Config;

@Config
public class PresetConstants {
    // 0init, 1high drop, 2high hold, 3mid, 4low, 5specimen grab, 6specimen setup, 7specimen attach, 8hang
    public static int[] slidePresets = {0, 4000, 4000, 0, 0, 0, 0, 0, 2000};
    // 0init, 1high drop, 2high hold, 3mid, 4low, 5specimen grab, 6specimen setup, 7specimen attach, 8hang
    public static int[] armThetaPresets = {0, 200, 100, 600, 700, 820, 300, 400, 0};
    // 0init, 1high drop, 2high hold, 3mid, 4low, 5specimen grab, 6specimen setup, 7specimen attach, 8hang
    public static double[] axis1Presets = {0.38, 0.5, 0.6, 0.57, 0.63, 1, 0.8, 0.8, 0.38};

    // STATES USES THIS: full left, half left, center, half right, full right
    public static int[] axis2Positions = {2, 1, 0, 3, 4};

    // STATES DOES NOT USE THIS: center, half left, full left, half right, full right
    public static double[] axis2Presets = {0.43, 0.595, 0.76, 0.265, 0.1}; // good
    // open, closed
    public static double[] clawPresets = {0.35, 0.26}; // good
}
