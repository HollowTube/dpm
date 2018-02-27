package ca.mcgill.ecse211.Localization;

public class LocalizationUSController implements UltrasonicController{
	private double bandMax;
	private double bandMin;
	
	public LocalizationUSController(double bandMin, double bandMax){
		this.bandMax = bandMax;
		this.bandMin = bandMin;
	}
	@Override
	public void processUSData(int distance){
		if(distance<30){
			Lab4.noisemaker.systemSound(0);
		}
	}
	
	@Override
	public int readUSDistance(){
		
		return 0;
	}

}
