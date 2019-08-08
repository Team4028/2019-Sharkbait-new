package org.usfirst.frc.team4028.robot.utilities;

import java.util.Iterator;
import java.util.LinkedList;

//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	====================================
//	1		TomB		29.Mar.2017	Implemented separate table for Auton 
//  2		TomB		11.Apr.2017 Implemented nth order polynomials to calc shooter values
//  3		TomB		13.Apr.2017	Implemented linear interpolation between shooter table entries
//------------------------------------------------------

public class ShooterTable {
	public static ShooterTable _instance = new ShooterTable();
	
	public static ShooterTable getInstance() {
		return _instance;
	}
	
	// define class level working variables
	private LinkedList<ShooterTableEntry> _teleopTable = null;
	private LinkedList<ShooterTableEntry> _autonTable = null;
	private int _currentIndex = 0;
	private int _indexCounter;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	private ShooterTable() {
		_teleopTable = LoadTable();
		_autonTable = LoadAutonTable();
		
		// set the initial value of the current index to the 1st one marked as default
		_currentIndex = 1;
		ShooterTableEntry ste;
		Iterator<ShooterTableEntry> itr = _teleopTable.iterator();
		while(itr.hasNext()) {
			ste = itr.next();
			if (ste.IsDefault == true) {
				_currentIndex = ste.Index;
				break;
			}
		}	
	}

	//============================================================================================
	// methods follow
	//============================================================================================
	// This method uses entirely dynamic values with a n order polynomial
	public ShooterTableEntry CalcShooterValues (double distanceInInches)
	{
		// y = -0.0000000013inches4 + 0.0000007088inches3 - 0.0001473231inches2 + 0.0148431960inches + 0.0216283716
		//double actuatorCalcValue = (-0.0000000013 * Math.pow(distanceInInches, 4)) 
		//							+ (0.0000007088 * Math.pow(distanceInInches, 3)) 
		//							+ (-0.0001473231 * Math.pow(distanceInInches, 2)) 
		//							+ (0.0148431960 * distanceInInches) 
		//							+ 0.0216283716;
		
		double actuatorCalcValue = 0.3;
		if(distanceInInches >= 72.0 && distanceInInches < 144.0)
		{
			actuatorCalcValue = ((distanceInInches - 72) * 0.00138889) + 0.57;
		}
		else if(distanceInInches >= 144.0)
		{
			actuatorCalcValue = ((distanceInInches - 144) * 0.00083333) + 0.67;
		}
		
		// Stage 1 = -0.0000754 inches^3 + 0.0022174 inches^2 - 4.7583458 inches - 2,633.7902098
		int stg1CalculatedRPM = (int) (Math.round(-0.0000754 * Math.pow(distanceInInches, 3)) 
												+ (0.0022174 * Math.pow(distanceInInches, 2)) 
												+ (-4.7583458 * distanceInInches) 
												- 2633.7902098);
		
		// Stage 2 = 0.0000757 inches^3 - 0.0571191 inches^2 + 2.7158189 inches - 2,633.6683317
		int stg2CalculatedRPM = (int) (Math.round( 0.0000757 * Math.pow(distanceInInches, 3)) 
												+ (-0.0571191 * Math.pow(distanceInInches, 2)) 
												+ (2.7158189 * distanceInInches) 
												- 2633.6683317);
		
		ShooterTableEntry ste = new ShooterTableEntry(_indexCounter++, 
														distanceInInches, 
														"Vision", 
														actuatorCalcValue, 
														stg1CalculatedRPM, 
														stg2CalculatedRPM, 
														false);
		
		return ste;
	}
	
	// This method uses linear interpolation between shooter table values
	public ShooterTableEntry CalcShooterValues2 (double distanceInInches)
	{
		ShooterTableEntry steBelow = null;
		ShooterTableEntry steAbove = null;
		ShooterTableEntry steCurrent = null;
		
		// loop thru and find the entries above and below the target distance
		Iterator<ShooterTableEntry> itr = _teleopTable.iterator();
		while(itr.hasNext()) {
			steCurrent = itr.next();
			
			// if target distance is longer, snapshot lower, continue looping
			if (steCurrent.DistanceInInches < distanceInInches) {
				steBelow = steCurrent;
				continue;
			}
			// if exactly same, snapshot both. stop looping
			else if (steCurrent.DistanceInInches == distanceInInches) {
				steBelow = steCurrent;
				steAbove = steCurrent;
				break;
			}
			// if longer, snapshot Above, stop looping
			else if (steCurrent.DistanceInInches > distanceInInches) {
				steAbove = steCurrent;
				break;
			}
		}
			
		ShooterTableEntry ste = null;
		
		// try to be robust about returning a value in all reasonable cases
		if(steBelow != null && steAbove != null)
		{
			// find the scale factor which is how far we are between the below & above ste
			double scaleFactor = (distanceInInches - steBelow.DistanceInInches) 
									/ (steAbove.DistanceInInches - steBelow.DistanceInInches);
			
			// calc the actuator value
			double actuatorCalcValue = steBelow.SliderPosition 
										+ (scaleFactor * (steAbove.SliderPosition - steBelow.SliderPosition));
			
			// round to int
			double stg1Adj = scaleFactor * (steAbove.Stg1MotorRPM - steBelow.Stg1MotorRPM);
			int stg1CalculatedRPM = steBelow.Stg1MotorRPM 
										+ (int)(Math.round(stg1Adj));
			
			// round to int
			double stg2Adj = scaleFactor * (steAbove.Stg2MotorRPM - steBelow.Stg2MotorRPM);
			int stg2CalculatedRPM = steBelow.Stg2MotorRPM 
										+ (int)(Math.round(stg2Adj));
			
			// build the return object
		    ste = new ShooterTableEntry(_indexCounter++, 
					distanceInInches, 
					"Table+Vision", 
					actuatorCalcValue, 
					stg1CalculatedRPM, 
					stg2CalculatedRPM, 
					false);
		}
		else if (steAbove != null) {
			ste = steAbove;
		}
		else {
			ste = steBelow;
		}
		
		return ste;
	}
	
	public ShooterTableEntry getNextEntry() {
		if(!get_IsAtUpperEntry()) {
			_currentIndex++;
		}
		
		return _teleopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getCurrentEntry() {
		return _teleopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getPreviousEntry() {
		if(!get_IsAtLowerEntry()) {
			_currentIndex--;
		}

		return _teleopTable.get(_currentIndex);
	}
	
	public ShooterTableEntry getAutonEntryForDistance(int targetDistanceInInches) {
		return getEntryForDistance(targetDistanceInInches, _autonTable);
	}
	
	public ShooterTableEntry getTelopEntryForDistance(int targetDistanceInInches) {
		return getEntryForDistance(targetDistanceInInches, _teleopTable);
	}
	
	// Return the correct SHooterTableEntry for a given distance
	// ShooterTableEntry ste = _shooterTable.getEntryForDistance(130);
	// System.out.println("..STE Entry: " + ste.Description);
	private ShooterTableEntry getEntryForDistance(int targetDistanceInInches, LinkedList<ShooterTableEntry> table) {
		// set the initial value of the current index to the 1st one marked as default
		int currentIndex = 1;
		ShooterTableEntry selectedSte = null;
		ShooterTableEntry previousSte = null;
		ShooterTableEntry currentSte = null;
		
		// loop thru and try to find the entries above and below the target distance
		Iterator<ShooterTableEntry> itr = table.iterator();
		while(itr.hasNext()) {
			currentSte = itr.next();
			
			// if target distance is longer, continue looping
			if (currentSte.DistanceInInches < targetDistanceInInches) {
				previousSte = currentSte;
				continue;
			}
			// else stop looping
			else if (currentSte.DistanceInInches >= targetDistanceInInches) {
				break;
			}
		}	
		
		// now decide which one to use
		if (currentSte == null) {
			selectedSte = table.get(1);	// should never happen!(just default to 1st entry
		}
		else if((previousSte == null) || (currentSte == previousSte)){
			selectedSte = currentSte;
		} else {
			// we have 2 options, need to decide which one is closer
			double previousSteDeltaDistance = Math.abs(targetDistanceInInches - previousSte.DistanceInInches);
			double thisSteDeltaDistance = Math.abs(currentSte.DistanceInInches - targetDistanceInInches);
			
			if(thisSteDeltaDistance <= previousSteDeltaDistance) {
				selectedSte = currentSte;
			} else {
				selectedSte = previousSte;
			}
		}
		
		return selectedSte;
	}
	
	//============================================================================================
	// properties follow
	//============================================================================================
	
	public Boolean get_IsAtUpperEntry() {
		if (_currentIndex == _teleopTable.size() - 1){
			return true;
		} else {
			return false;
		}
	}
	
	public Boolean get_IsAtLowerEntry() {
		if (_currentIndex == 0){
			return true;
		} else {
			return false;
		}
	}
	
	//============================================================================================
	// helpers follow
	//============================================================================================
	// create a linked list
	private LinkedList<ShooterTableEntry> LoadTable() {

		LinkedList<ShooterTableEntry> table = new LinkedList<ShooterTableEntry>();
		
		_indexCounter = 0;
		//======================================================================================
		//									Position	Inches Desc	Slider	Stg1  Stg2  Is Default?
		//======================================================================================
		// Worlds
		/*
		table.add(new ShooterTableEntry(_indexCounter++, 84, " 7ft", .53, -3125, -2825, false));
		table.add(new ShooterTableEntry(_indexCounter++, 108, " 9ft", .60, -3250, -2950, false));
		table.add(new ShooterTableEntry(_indexCounter++, 132, "11ft", .62, -3400, -3100, true));
		table.add(new ShooterTableEntry(_indexCounter++, 156, "13ft", .65, -3600, -3300, true));
		table.add(new ShooterTableEntry(_indexCounter++, 180, "15ft", .68, -3750, -3450, false));
		*/
		
		// Cleveland
		//table.add(new ShooterTableEntry(_indexCounter++, 84, " 7ft", .59, -3067, -2767, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 108, " 9ft", .59, -3400, -3100, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 132, "11ft", .65, -3400, -3100, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 156, "13ft", .68, -3617, -3317, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 180, "15ft", .70, -3850, -3550, false));
		
		// Mod Cleveland (Sat)
		table.add(new ShooterTableEntry(_indexCounter++, 84, " 7ft", .53, -3017, -2717, false));
		table.add(new ShooterTableEntry(_indexCounter++, 108, " 9ft", .59, -3225, -2925, false));
		table.add(new ShooterTableEntry(_indexCounter++, 132, "11ft", .65, -3350, -3050, false));
		table.add(new ShooterTableEntry(_indexCounter++, 156, "13ft", .68, -3517, -3217, false));
		table.add(new ShooterTableEntry(_indexCounter++, 180, "15ft", .70, -3800, -3500, false));
		
		// Reference
		//table.add(new ShooterTableEntry(_indexCounter++, 36, " 3ft", .40, -2800, -2600, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 48, " 4ft", .46, -2867, -2633, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 60, " 5ft", .51, -2933, -2667, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 72, " 6ft", .57, -3000, -2700, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 84, " 7ft", .59, -3067, -2767, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 96, " 8ft", .60, -3133, -2833, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 108, " 9ft", .59, -3400, -3100, false));
		// old table.add(new ShooterTableEntry(_indexCounter++, 108, " 9ft", .62, -3200, -2900, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 120, "10ft", .64, -3300, -3000, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 132, "11ft", .65, -3400, -3100, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 144, "12ft", .67, -3500, -3200, true));
		//table.add(new ShooterTableEntry(_indexCounter++, 156, "13ft", .68, -3617, -3317, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 168, "14ft", .69, -3733, -3433, false));
		//table.add(new ShooterTableEntry(_indexCounter++, 180, "15ft", .70, -3850, -3550, false));
		
		return table;
	}

	private LinkedList<ShooterTableEntry> LoadAutonTable() {
		
		LinkedList<ShooterTableEntry> table = new LinkedList<ShooterTableEntry>();
		
		int indexCounter = 0;
		//======================================================================================
		//									Position	Inches Desc	Slider	Stg1  Stg2  Is Default?
		//======================================================================================
		
		table.add(new ShooterTableEntry(indexCounter++, 114, "9' 6ft", .60, -3375, -3075, false));
		table.add(new ShooterTableEntry(indexCounter++, 156, "13ft", .68, -3617, -3317, false));
		table.add(new ShooterTableEntry(indexCounter++, 147, "12' 3ft", .63, -3479, -3185, false));
		table.add(new ShooterTableEntry(indexCounter++, 150, "12' 6ft", .643, -3550, 3250, false));
		table.add(new ShooterTableEntry(indexCounter++, 172, "14' 4ft", .69, -3771, -3471, false));
		table.add(new ShooterTableEntry(indexCounter++, 175, "14' 7ft", .69, -3811, -3511, false));
		
		return table;
	}
}