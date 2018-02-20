package ca.mcgill.ecse211.lab4;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.exception.MathArithmeticException;
import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.TextLCD;

public class LightPollerColor {

	private static final float COLOR_THRESHOLD = 0.2f;
	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;
	private float[] desiredColor;

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	// private Color red_block = new Color(0,0,0,0,0,0);
	// private Color blue_block = new Color(0,0,0,0,0,0);
	// private Color yellow_block = new Color(0,0,0,0,0,0);
	private Color orange = new Color(0.164809809f, 0.128121575f, 0.096505883f, 0.1574731f, 0.1548986f,
			0.15302903f);

	public LightPollerColor(SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
	}

	public void calibrate() {
		lt.fetchSample(ltdata, 0);
		for (int i = 0; i < 500; i++) {
			lt.fetchSample(ltdata, 0);
			System.out.println(ltdata[0] + "\t" + ltdata[1] + "\t" + ltdata[2]);
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		Sound.beep();
	}

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

	private double getProbability(Color color, float[] reading) {
		float sum_prob = 0, red_dist = 0, green_dist = 0, blue_dist = 0;
		float red_prob = 0, blue_prob = 0, green_prob = 0;

		red_dist = Math.abs(color.red_mean - reading[0]);
		green_dist = Math.abs(color.green_mean - reading[1]);
		blue_dist = Math.abs(color.blue_mean - reading[2]);

		System.out.println("red dist: " + red_dist);
		System.out.println("green dist: " + green_dist);
		System.out.println("blue dist: " + blue_dist);

		System.out.println("red mean: " + color.red_mean);
		System.out.println("green mean: " + color.green_mean);
		System.out.println("blue mean: " + color.blue_mean);

		// sum_prob += 1-color.red.probability(color.red_mean-red_dist,
		// color.red_mean+red_dist);
		// sum_prob += 1-color.green.probability(color.green_mean-green_dist,
		// color.green_mean+green_dist);
		// sum_prob += 1-color.red.probability(color.blue_mean-blue_dist,
		// color.blue_mean+blue_dist);

		red_prob = (float) (1 - color.red.probability(color.red_mean - red_dist, color.red_mean + red_dist));
		green_prob = (float) (1
				- (color.green.probability(color.green_mean - green_dist, color.green_mean + green_dist)));
		blue_prob = (float) (1 - (color.red.probability(color.blue_mean - blue_dist, color.blue_mean + blue_dist)));

		// lcd.drawString("Red: " + Float.toString(red_dist), 0, 0);
		// lcd.drawString("Green: " +Float.toString(green_dist), 0, 1);
		// lcd.drawString("Blue: " +Float.toString(blue_dist), 0, 2);

		System.out.println("red prob: " + red_prob);
		System.out.println("green prob: " + green_prob);
		System.out.println("blue prob: " + blue_prob);

		lcd.drawString("Red: " + Float.toString(red_prob), 0, 0);
		lcd.drawString("Green: " + Float.toString(green_prob), 0, 1);
		lcd.drawString("Blue: " + Float.toString(blue_prob), 0, 2);

		return sum_prob / 3;
	}

	public void detectColor() {
		double prob_red = 0, prob_green = 0, prob_blue = 0, prob_orange = 10;
		double max_prob;
		lt.fetchSample(ltdata, 0);
		float[] reading = getAverageMeasurement();

		System.out.println("red read: " + reading[0]);
		System.out.println("green read: " + reading[1]);
		System.out.println("blue read: " + reading[2]);

		// prob_red = getProbability(red_block, reading);
		// prob_green = getProbability(blue_block, reading);
		// prob_blue = getProbability(yellow_block, reading);
		prob_orange = getProbability(orange, reading);

		// max_prob = Math.max(Math.max(prob_blue, prob_green), prob_red);

		lcd.drawString("Red: " + Float.toString(reading[0]), 0, 3);
		lcd.drawString("Green: " + Float.toString(reading[1]), 0, 4);
		lcd.drawString("Blue: " + Float.toString(reading[2]), 0, 5);

		// lcd.drawString("Orange: " +Double.toString(prob_orange), 0, 3);

		// if(max_prob < 0.75) {
		// System.out.println("unknown color");
		// }
		//
		// else if(max_prob == prob_red) {
		// System.out.println("its Red with a probability of " + prob_red);
		// }
		// else if(max_prob == prob_green) {
		// System.out.println("its Green with a probability of " + prob_green);
		// }
		// else {
		// System.out.println("its Blue with a probability of " + prob_blue);
		// }
	}
}
