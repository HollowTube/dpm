package ca.mcgill.ecse211.lab4;

import lejos.robotics.SampleProvider;

public class LightPoll {
	
	private SampleProvider lt;
	private float[] ltdata;
	private int lightVal;
	
	private int prev_light;
	private int current_light;
	
	private boolean first_time = true;
	
	public LightPoll (SampleProvider lt, float[] ltdata) {
		this.lt = lt;
		this.ltdata = ltdata;
		
	}
	
	public int getValue() {
		lt.fetchSample(ltdata, 0);
		lightVal = (int) (ltdata[0] * 100);
		return lightVal;
		
	}
	public boolean lessThan(int threshold) {
		current_light = getValue();
		
		if(current_light<  threshold) {
			return true;
		}
		else return false;
	}
	
	public boolean falling(int threshold) {
		boolean edge;
		current_light = getValue();
		
		if(first_time) {
			prev_light = current_light;
			first_time = false;
		}
		
		if(current_light < threshold && prev_light > threshold) edge = true;
		else edge = false;
		prev_light = current_light;
		return edge;
	}
	
	public boolean rising(int threshold) {
		boolean edge;
		current_light = getValue();
		
		if(first_time) {
			prev_light = current_light;
			first_time = false;
		}
		
		if(current_light > threshold && prev_light < threshold) edge = true;
		else edge = false;
		prev_light = current_light;
		return edge;
	}
	
	public void main (String args[]) throws InterruptedException {
		while (true) {
			falling(20);
			Thread.sleep(20);
		}
	}
	
	
}
