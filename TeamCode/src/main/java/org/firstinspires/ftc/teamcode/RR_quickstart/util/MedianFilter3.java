package org.firstinspires.ftc.teamcode.RR_quickstart.util;


public class MedianFilter3 {

	public double m1 = 0;
	public double m2 = 0;
	public double m3 = 0;
	public double m4 = 0;
	public double m5 = 0;

	public double estimate(double measurement) {
		m1 = m2;
		m2 = m3;
		m3 = m4;
		m4 = m5;
		m5 = measurement;
		double[] data = {m1, m2, m3, m4, m5};

		for (int i = 0; i < data.length; ++i) {
			for (int j = 0; j < data.length; ++j) {
				if (data[i] > data[j]) {
					// swap
					double temp = data[i];
					data[i] = data[j];
					data[j] = temp;
				}
			}
		}

		return data[2];

	}
}
