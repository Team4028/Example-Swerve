/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DecimalFormat;
import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.RobotMap;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.math.Vector2;

/**
 * Add your docs here.
 */
public class util {
    public static double inchesToMeters(double inches){
        return inches / 39.37;
    }

    public static double metersToInches(double meters){
        return meters * 39.37;
	}
	
	public static Supplier<Boolean> getTrueSupplier(){
		return () -> true;
	}

	public static Supplier<Boolean> getFalseSupplier(){
		return () -> false;
	}

    public static DataLogger setupLogging(String mode) {
		DataLogger dataLogger;
				
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.PRIMARY_LOG_FILE_PATH);
		Path alternatePath = Paths.get(RobotMap.ALTERNATE_LOG_FILE_PATH);
    	if (Files.exists(path)) {
    		try {
				dataLogger = new DataLogger(RobotMap.PRIMARY_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + RobotMap.PRIMARY_LOG_FILE_PATH);
			}
    	}
    	else if (Files.exists(alternatePath)) {
    		try {
				dataLogger = new DataLogger(RobotMap.ALTERNATE_LOG_FILE_PATH, mode);
					    		
	    		System.out.println("..Logging enabled to: " + dataLogger.getLogFilePathName());
			} catch (IOException e) {
				e.printStackTrace();
				
	    		dataLogger = null;
	    		
	    		System.out.println("..Error configuring Logging to: " + RobotMap.ALTERNATE_LOG_FILE_PATH);
    		}
    	} else {
    		dataLogger = null;
    		
    		System.out.println("..Logging Disabled!");
    	}
    	return dataLogger;
	}

	public static double inchesToFeet(double inches){
		return inches / 12.;
	}

	public static double feetToInches(double feet){
		return feet * 12.;
	}

	public static double iversonBrackets(boolean bool){
		return bool ? 1: 0;
	}

	public static Translation2d transFromAngle(double angDegrees){
		return new Translation2d(Math.cos(Math.toRadians(angDegrees)), Math.sin(Math.toRadians(angDegrees)));
	}

	public static boolean isInRange(double val, double lower, double upper){
		return (val >= lower) && (val <= upper);
	}

	public static double getAngleDegrees(Translation2d t){
		return Math.toDegrees(Math.atan2(t.getY(), t.getX()));
	}
	public static Vector2 i_hat = new Vector2(1, 0);
	public static Vector2 j_hat = new Vector2(0, 1);

	public static Double roundNum(double val, int decimalPlaces){
		return (double)(Math.round(val * Math.pow(10, decimalPlaces)) / Math.pow(10, decimalPlaces));
	}

	public static String getRoundedString(double val, int decimalPlaces){
		double rounded = roundNum(val, decimalPlaces);
		String keyStr = decimalPlaces > 0 ? "0." : "0";
		for (int ind = 0; ind < decimalPlaces; ind++){
			keyStr += "0";
		}
		DecimalFormat df = new DecimalFormat(keyStr);
		return df.format(rounded);
	}

	public static double lin_int(double x0, double y0, double x1, double y1, double x_star){
		if (x_star <= x0){
			return y0;
		} else if (x_star >= x1) {
			return y1;
		} else {
			double slope = (y1 - y0) / (x1 - x0);
			return y0 + (x_star - x0) * slope;
		}
	}
}
