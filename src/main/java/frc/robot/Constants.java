// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Motor Controller Constants - Mecanum Drive
	public static final int BACK_LEFT_MOTOR_ID = 3;
	public static final int BACK_RIGHT_MOTOR_ID = 4;
	public static final int FRONT_LEFT_MOTOR_ID = 1;
	public static final int FRONT_RIGHT_MOTOR_ID = 2;


    //Speed multiplier for more accurate driving in mecanum
	public static final double MECANUM_TURN_MULTIPLIER = .8;

    //Motor speeds
	public static final double AUTO_DRIVE_SPEED = 1.0;
	public static double DRIVE_SPEED = 1.0;
	public static final double DRIVE_SPEED_FAST = 1.0;
	public static final double DRIVE_SPEED_SLOW = 0.5;



	
    
	//commented out because the bot does not have encoders yet
	//Encoder config values
	/*public static final int kEncoderPPR = 4096;
	public static final boolean kSensorPhase = true;
	public static final boolean kMotorInvert = true;
	public static final int kTimeoutMs = 10;
	public static final int kPIDLoopIdx = 0;
	public static final int ARM_ENCODER_START_POS = 0;*/





	//PID values for mecanum
	public static final double kP_Straight = 0.05;
	public static final double kP_Slew = 1;
	public static final double kP_Turn = .5;
	public static final double kI_Straight = 0.0;
	public static final double kI_Slew = 0.0;
	public static final double kI_Turn = 0.0;
	public static final double kD_Straight = 0.004;
	public static final double kD_Slew = 0.00;
	public static final double kD_Turn = 0.02;
	//Drive control port IDs
	public static final int XBOX_PORT = 0;
	public static final int LEFT_JOYSTICK = 1;
	public static final int RIGHT_JOYSTICK = 2;
}
