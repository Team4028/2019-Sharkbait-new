package org.usfirst.frc.team4028.robot.vision;

/* 
 * DTO (data transfer object) This class represents the data retrieved from the RoboRealm API
 */
public class RawImageData {
	public long Timestamp;
	public String CameraType;
    public double ResponseTimeMSec;
    public int BlobCount;
    public double SouthWestX;
    public double SouthWestY;
    public double SouthEastX;
    public double SouthEastY;

    public double HighMiddleY;
    
    public double EstimatedDistance;

    public Dimension FOVDimensions;
    
    public boolean IsVisionDataValid;
    public int BadDataCounter;
    
    public double FovCenterToTargetXAngleRawDegrees;
}