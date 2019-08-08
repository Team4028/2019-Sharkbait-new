package org.usfirst.frc.team4028.robot.autonRoutines;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the "Do Nothing" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//------------------------------------------------------
//=====> For Changes see Sebas
public class DoNothing {
	//============================================================================================
	// constructor follows
	//============================================================================================
	public DoNothing() {}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		DriverStation.reportWarning("===== Entering DoNothing Auton =====", false);
	}
	
	public boolean ExecuteRentrant() {
		// cleanup
		DriverStation.reportWarning("===== Completed DoNothing Auton =====", false);
		
		return false; 
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() { return false; }
}