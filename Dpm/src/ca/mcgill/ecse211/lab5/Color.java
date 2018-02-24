package ca.mcgill.ecse211.lab5;
import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.exception.MathArithmeticException;

public class Color {
	public float getRed_mean() {
		return red_mean;
	}

	public void setRed_mean(float red_mean) {
		this.red_mean = red_mean;
	}

	public float getGreen_mean() {
		return green_mean;
	}

	public void setGreen_mean(float green_mean) {
		this.green_mean = green_mean;
	}

	public float getBlue_mean() {
		return blue_mean;
	}

	public void setBlue_mean(float blue_mean) {
		this.blue_mean = blue_mean;
	}

	public float getRed_sigma() {
		return red_sigma;
	}

	public void setRed_sigma(float red_sigma) {
		this.red_sigma = red_sigma;
	}

	public float getBlue_sigma() {
		return blue_sigma;
	}

	public void setBlue_sigma(float blue_sigma) {
		this.blue_sigma = blue_sigma;
	}

	public float getGreen_sigma() {
		return green_sigma;
	}

	public void setGreen_sigma(float green_sigma) {
		this.green_sigma = green_sigma;
	}

	public NormalDistribution getRed() {
		return red;
	}

	public void setRed(NormalDistribution red) {
		this.red = red;
	}

	public NormalDistribution getGreen() {
		return green;
	}

	public void setGreen(NormalDistribution green) {
		this.green = green;
	}

	public NormalDistribution getBlue() {
		return blue;
	}

	public void setBlue(NormalDistribution blue) {
		this.blue = blue;
	}

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
