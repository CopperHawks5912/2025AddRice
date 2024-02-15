package frc.robot;

public class Gains {
    public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	public Gains(double p, double i, double d, double f, int iZone, double peakOutput){
		kP = p;
		kI = i;
		kD = d;
		kF = f;
		kIzone = iZone;
		kPeakOutput = peakOutput;
	}   
}
