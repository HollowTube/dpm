package ca.mcgill.ecse211.main_package;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.LCD;

/**
 * This class uses a probabilistic normal distribution model to detect the right
 * colored blocks. It allows identification of the correct block by choosing the
 * highest probable one as the right one.
 * 
 * @author Tritin
 *
 */
public class LightPollerColor {

	private SampleProvider lt;
	private float[] ltdata;
	public MotorControl motorcontrol;

	private Color red_block = new Color(0.16767974f, 0.017133795f, 0.015695887f, 0.3038505f, 0.04516072f, 0.02923487f,
			"red");
	private Color blue_block = new Color(0.035873442f, 0.058556151f, 0.059581107f, 0.04077874f, 0.05821204f,
			0.04725411f, "blue");
	private Color yellow_block = new Color(0.24775123f, 0.141819854f, 0.0246875f, 0.5973893f, 0.5888729f, 0.05106323f,
			"yellow");
	private Color white_block = new Color(0.244844f, 0.226581879f, 0.135455755f, 0.32806904f, 0.33583433f,
			0.05739245f, "white");
	ArrayList<Color> colors = new ArrayList<Color>();

	/**
	 * LightPollerColor class constructor. It is comprised of the SampleProvider
	 * sample of the sensor and its value.
	 * 
	 * @param lt
	 *            Light sensor sample
	 * @param ltdata
	 *            Value of sample
	 */
	public LightPollerColor(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
	}

	/**
	 * Method to calibrate the light sensor for each different colored block. Each
	 * time it is called, it collects data on the RGB values for a specific block.
	 * This will not be used once calibration in the right light condition is
	 * finished.
	 * 
	 */
	public void calibrate() {
		lt.fetchSample(ltdata, 0);

		for (int i = 0; i < 2; i++) {

			lt.fetchSample(ltdata, 0);
			System.out.println(ltdata[0] + "\t" + ltdata[1] + "\t" + ltdata[2]);
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

//		Sound.beep();
	}

	/**
	 * This method obtains the average out of 10 samples for RGB values detected by
	 * the light sensor. This is used when trying to identify the right colored
	 * block.
	 * 
	 * @return Average array [avg_red, avg_green, avg_blue]
	 */
	private float[] getAverageMeasurement() {
		float[] avg_values = { 0, 0, 0 };
		float red_sum = 0, green_sum = 0, blue_sum = 0;

		for (int i = 0; i < 10; i++) {
			lt.fetchSample(ltdata, 0);
			red_sum += ltdata[0];
			green_sum += ltdata[1];
			blue_sum += ltdata[2];
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		avg_values[0] = red_sum / 10;
		avg_values[1] = green_sum / 10;
		avg_values[2] = blue_sum / 10;
		return avg_values;
	}

	/**
	 * This method uses the mean and standard deviation found for each color during
	 * calibration to obtain the probability for each color. It uses a normal
	 * distribution model for each RGB value. Then, it compares the reading with
	 * each distribution and finds an average probability.
	 * 
	 * 
	 * @param color
	 *            Block Color
	 * @param reading
	 *            Reading obtained by the light sensor
	 * @return Average probability of Block Color from RGB values
	 */
	private double getProbability(Color color, float[] reading) {
		float red_dist = 0, green_dist = 0, blue_dist = 0;
		float red_prob = 0, blue_prob = 0, green_prob = 0;

		red_dist = Math.abs(color.red_mean - reading[0]);
		green_dist = Math.abs(color.green_mean - reading[1]);
		blue_dist = Math.abs(color.blue_mean - reading[2]);


		red_prob = (float) (1 - color.red.probability(color.getRed_mean() - red_dist, color.getRed_mean() + red_dist));
		green_prob = (float) (1
				- (color.green.probability(color.getGreen_mean() - green_dist, color.getGreen_mean() + green_dist)));
		blue_prob = (float) (1
				- (color.red.probability(color.getBlue_mean() - blue_dist, color.getBlue_mean() + blue_dist)));
		

		// returns the average probability of each probability
		return (red_prob + green_prob + blue_prob) / 3;
	}

	/**
	 * This method displays the color detected, either red, blue, yellow or white.
	 * It uses the reading from the light sensor and finds the probability of that
	 * reading with each color. It then chooses the largest one by comparing each
	 * value once and retaining the larger one each team. Whichever color has the
	 * maximum probability, the LCD displays it.
	 * 
	 */
	public void detectColor() {
		double prob_red = 0, prob_blue = 0, prob_yellow = 0, prob_white = 0;
		double max_prob;
		lt.fetchSample(ltdata, 0);
		float[] reading = getAverageMeasurement();
		

		prob_yellow = getProbability(yellow_block, reading);
		prob_blue = getProbability(blue_block, reading);
		prob_red = getProbability(red_block, reading);
		prob_white = getProbability(white_block, reading);

		System.out.println("yellow " + prob_yellow);
		System.out.println("blue " + prob_blue);
		System.out.println("red " + prob_red);
		System.out.println("white " + prob_white);

		max_prob = Math.max(Math.max(Math.max(prob_blue, prob_yellow), prob_red), prob_white);

		if (max_prob == prob_red) {
			LCD.drawString(("its Red "), 0, 1);
		} else if (max_prob == prob_blue) {
			LCD.drawString(("its Blue  "), 0, 1);
		} else if (max_prob == prob_yellow) {
			LCD.drawString(("its Yellow  "), 0, 1);
		} else {
			LCD.drawString(("its White "), 0, 1);
		}
	}

	/**
	 * If the color scanned is the target color, the robot will beep twice and
	 * return true else will beep once and return false.
	 * 
	 * @param name
	 *            Target block color
	 * @return True if TB is found, false otherwise
	 */
	public boolean target_found(String name) {
		double prob_red = 0, prob_blue = 0, prob_yellow = 0, prob_white = 0, max_prob = 0;
		String max_color;

		lt.fetchSample(ltdata, 0);
		float[] reading = getAverageMeasurement();

		prob_yellow = getProbability(yellow_block, reading);
		prob_blue = getProbability(blue_block, reading);
		prob_red = getProbability(red_block, reading);
		prob_white = getProbability(white_block, reading);

//		System.out.println("yellow " + prob_yellow);
//		System.out.println("blue " + prob_blue);
//		System.out.println("red " + prob_red);
//		System.out.println("white " + prob_white);

		max_prob = Math.max(Math.max(Math.max(prob_blue, prob_yellow), prob_red), prob_white);

		if (max_prob == prob_red) {
			max_color = "red";
		} else if (max_prob == prob_blue) {
			max_color = "blue";
		} else if (max_prob == prob_yellow) {
			max_color = "yellow";
		} else {
			max_color = "white";
		}
		if (max_color.equals(name)) {
			Sound.beep();
			Sound.beep();
			colors.clear();
			return true;
		} else {
			Sound.beep();
			colors.clear();
			return false;
		}

	}

	/**
	 * This method makes sure that that the colored block chosen is the one with
	 * highest probability.
	 * 
	 * @param list
	 *            Array with each probability
	 * @return Color having maximum probability
	 */
	public Color max_color(ArrayList<Color> list) {
		Color max_color = list.get(0);
		for (int i = 0; i < 4; i++) {
			if (max_color.getProbability() < list.get(i).getProbability()) {
				max_color = list.get(i);
			}
		}
		return max_color;
	}

}
