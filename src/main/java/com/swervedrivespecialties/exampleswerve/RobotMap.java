package com.swervedrivespecialties.exampleswerve;

public class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; // CAN

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 3; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 4; // CAN

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6; // CAN

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 8; // CAN

    public static final int KICKER_TALON = 11;
    public static final int SHOOTER_SLAVE_NEO  = 9; 
    public static final int SHOOTER_MASTER_NEO  = 10; 

    //infeed stuffs
    public static final int CONVEYOR_MOTOR = 12;
    public static final int SINGULATOR_MOTOR = 13;
    public static final int INFEED_MOTOR = 14;

    //Climber
    public static final int CLIMBER_MOTOR = 15;

    public static final int PRE_CONVEYOR_SENSOR = 1;
    public static final int PRE_SHOOTER_SENSOR = 3;
    public static final int POST_SINGULATOR_SENSOR = 0;
    public static final int MID_CONVEYOR_SENSOR = 2;

    //Camera Addresses
    public static final String RASPBERRY_PI_INFEED_CAMERA = "http://10.40.28.15:1181/stream.mjpg";
    public static final String RASPBERRY_PI_OTHER_CAMERA = "http://10.40.28.15:1182/stream.mjpg";
    public static final String LIMELIGHT_CAMERA = "http://10.40.28.13:5800/";


    	// Logging
        // this is where the USB stick is mounted on the RoboRIO filesystem.  
        // You can confirm by logging into the RoboRIO using WinSCP
        public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
        public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
}