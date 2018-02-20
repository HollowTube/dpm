package ca.mcgill.ecse211.lab4;
import org.apache.commons.math3.distribution.NormalDistribution;

public class Color {
	float red_mean, green_mean, blue_mean, red_sigma, blue_sigma, green_sigma;
	public NormalDistribution red,green,blue;

	public Color(float red_mean, float green_mean, float blue_mean, float red_sigma, float blue_sigma,
			float green_sigma) {
		
		super();
		this.red_mean = red_mean;
		this.green_mean = green_mean;
		this.blue_mean = blue_mean;
		this.red_sigma = red_sigma;
		this.blue_sigma = blue_sigma;
		this.green_sigma = green_sigma;
		
		red = new NormalDistribution(red_mean,red_sigma);
		blue = new NormalDistribution(blue_mean,blue_sigma);
		green = new NormalDistribution(green_mean,green_sigma);
	}
	
	
		
}
