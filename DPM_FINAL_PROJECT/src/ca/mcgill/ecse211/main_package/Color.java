package ca.mcgill.ecse211.main_package;

import org.apache.commons.math3.distribution.NormalDistribution;

/**
 * This class is used to store the values for each colored block sample normal distributions,
 * mean and standard deviation. It will then bring easier access to the data
 * when performing the identification of the block.
 * 
 * @author Tritin
 * 
 */

public class Color {
	
	/**
	 * Get method for red mean value.
	 * 
	 * @return red mean value
	 */
	public float getRed_mean() {
		return red_mean;
	}

	/**
	 * Set method for red mean value.
	 * 
	 * @param red_mean
	 */
	public void setRed_mean(float red_mean) {
		this.red_mean = red_mean;
	}

	/**
	 * Get method for green mean value.
	 * 
	 * @return green mean value
	 */
	public float getGreen_mean() {
		return green_mean;
	}

	/**
	 * Set method for green mean value.
	 * 
	 * @param green_mean value
	 */
	public void setGreen_mean(float green_mean) {
		this.green_mean = green_mean;
	}

	/**
	 * Get method for blue mean value.
	 * 
	 * @return blue_mean value
	 */
	public float getBlue_mean() {
		return blue_mean;
	}

	/**
	 * Set method for blue mean value.
	 * 
	 * @param blue_mean value
	 */
	public void setBlue_mean(float blue_mean) {
		this.blue_mean = blue_mean;
	}

	/**
	 * Get method for red sigma
	 * 
	 * @return red sigma
	 */
	public float getRed_sigma() {
		return red_sigma;
	}

	/**
	 * Set method for red sigma
	 * 
	 * @param red_sigma
	 */
	public void setRed_sigma(float red_sigma) {
		this.red_sigma = red_sigma;
	}

	/**
	 * Get method for blue sigma
	 * 
	 * @return
	 */
	public float getBlue_sigma() {
		return blue_sigma;
	}

	/**
	 * Set method for blue sigma
	 * 
	 * @param blue_sigma
	 */
	public void setBlue_sigma(float blue_sigma) {
		this.blue_sigma = blue_sigma;
	}

	/**
	 * Get method for green sigma
	 * 
	 * @return
	 */
	public float getGreen_sigma() {
		return green_sigma;
	}

	/**
	 * Set method for green sigma
	 * 
	 * @param green_sigma
	 */
	public void setGreen_sigma(float green_sigma) {
		this.green_sigma = green_sigma;
	}

	/**
	 * Get method for NormalDistribution object of red
	 * 
	 * @return NormalDistribution for red
	 */
	public NormalDistribution getRed() {
		return red;
	}

	/**
	 * Set method for NormalDistribution object of red
	 * 
	 * @param red
	 */
	public void setRed(NormalDistribution red) {
		this.red = red;
	}

	/**
	 * Get method for NormalDistribution object of green
	 * 
	 * @return green
	 */
	public NormalDistribution getGreen() {
		return green;
	}

	/**
	 * Set method for NormalDistribution objet of green
	 * 
	 * @param green
	 */
	public void setGreen(NormalDistribution green) {
		this.green = green;
	}

	/**
	 * Get method for NormalDistribution object of blue
	 * 
	 * @return
	 */
	public NormalDistribution getBlue() {
		return blue;
	}

	/**
	 * Set method for NormalDistribution object of blue
	 * 
	 * @param blue
	 */
	public void setBlue(NormalDistribution blue) {
		this.blue = blue;
	}

	/**
	 * Set method for probability
	 * 
	 * @param double prob_yellow
	 */
	public void setProbability(double prob_yellow) {
		this.probability = prob_yellow;
	}

	/**
	 * Get method for probability
	 * 
	 * @return
	 */
	public double getProbability() {
		return this.probability;
	}

	float red_mean, green_mean, blue_mean, red_sigma, blue_sigma, green_sigma;
	double probability;
	String name;
	public NormalDistribution red, green, blue;

	/**
	 * This is the Color class constructor.
	 * 
	 * @param red_mean Red Mean
	 * @param green_mean Green Mean
	 * @param blue_mean Blue Mean
	 * @param red_sigma Red Standard deviation
	 * @param blue_sigma Blue Standard deviation
	 * @param green_sigma Green Standard deviation
	 * @param name Color
	 */
	public Color(float red_mean, float green_mean, float blue_mean, float red_sigma, float blue_sigma,
			float green_sigma, String name) {

		super();
		this.red_mean = red_mean;
		this.green_mean = green_mean;
		this.blue_mean = blue_mean;
		this.red_sigma = red_sigma;
		this.blue_sigma = blue_sigma;
		this.green_sigma = green_sigma;
		this.name = name;

		red = new NormalDistribution(red_mean, red_sigma);
		blue = new NormalDistribution(blue_mean, blue_sigma);
		green = new NormalDistribution(green_mean, green_sigma);
	}

}
