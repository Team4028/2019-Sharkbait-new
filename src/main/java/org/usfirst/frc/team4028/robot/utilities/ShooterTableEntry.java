package org.usfirst.frc.team4028.robot.utilities;

public class ShooterTableEntry { 
	// define class level working variables
	public final int Index;
	public final double DistanceInInches;
	public final String Description;
	public final double SliderPosition;
	public final int Stg1MotorRPM;
	public final int Stg2MotorRPM;
	public final boolean IsDefault;

	//============================================================================================
	// constructors follow
	//============================================================================================
	public ShooterTableEntry(int index, double distanceInInches, String description, double sliderPosition, int stg1MotorRPM, int stg2MotorRPM, boolean isDefault) {
		Index = index;
		DistanceInInches = distanceInInches;
		Description = description;
		SliderPosition = sliderPosition;
		Stg1MotorRPM = stg1MotorRPM;
		Stg2MotorRPM = stg2MotorRPM;
		IsDefault = isDefault;
	}
}