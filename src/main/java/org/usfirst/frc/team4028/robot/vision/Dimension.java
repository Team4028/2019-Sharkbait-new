package org.usfirst.frc.team4028.robot.vision;

/* Small class used when both width and height are needed as return values */
public class Dimension {
  public double width;
  public double height;

  public Dimension() {};
  
  public Dimension(double w, double h) {
    width=w;
    height=h;
  }
}