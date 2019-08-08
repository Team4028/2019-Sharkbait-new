package org.usfirst.frc.team4028.robot.util;

/* 
 * Holds the beefiest, and therefore the best, methods pertaining
 * to mathematical computations used on the robot. Partially inspired
 * by Beefulus
 * 
 *          (__)
 *        	(oo)                      
 *	 /-------\/                   
 *  / |     ||                                  
 *    ||----||
 *    ^^    ^^
 */

public class BeefyMath {
	private static final double ENCODER_ROTATIONS_PER_DEGREE = 77.371/3600; // Check if this needs to be in encoder rotations
	public static double arctan(double heading) {
		double degrees = Math.toDegrees(Math.atan(heading));
		return degrees;
    }
  
    public static double degreesToEncoderRotations(double degrees) {
	    double encoderRotations = ENCODER_ROTATIONS_PER_DEGREE * degrees;
	    return encoderRotations;
    }
}