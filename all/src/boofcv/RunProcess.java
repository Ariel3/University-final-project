package boofcv;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrack;
import boofcv.abst.feature.tracker.PointTrackerKltPyramid;
import boofcv.alg.filter.derivative.GImageDerivativeOps;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTracker;
import boofcv.gui.feature.VisualizeFeatures;
import boofcv.gui.image.ImageGridPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.SimpleImageSequence;
import boofcv.io.video.VideoMjpegCodec;
import boofcv.io.wrapper.images.JpegByteImageSequence;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageSingleBand;
import georegression.struct.point.Point2D_F64;
import gui.guiShowTrack;

public class RunProcess<T extends ImageSingleBand, D extends ImageSingleBand> {
	SimpleImageSequence sequenceL, sequenceR;
	Maping maping;
	double xxx = 0, yyy = 0, degree = 45;
	// type of input image
	Class<T> imageType;
	Class<D> derivType;
	Graphics2D gR, gL;
	BufferedImage origR, origL;
	// tracks point features inside the image
	PointTrackerKltPyramid<T, D> trackerL, trackerR;
	List<FeatureIdPoint> SecTrack;
	ImageGridPanel gui;
	String Dir;
	FileWriter fw;
	BufferedWriter bw;
	guiShowTrack newGui;
	BufferedImage imageArrow;
	
	// constructor
	public RunProcess(Class<T> imageType, double distBetweenCams, double camreaLen, String projectName, String dir,
			String leftCamera, String rightCamera) throws IOException {
		Dir = dir;
		// loading Video's
		VideoMjpegCodec codec = new VideoMjpegCodec();
		List<byte[]> dataL = codec.read(new FileInputStream(dir + leftCamera));
		sequenceL = new JpegByteImageSequence(imageType, dataL, true);
		List<byte[]> dataR = codec.read(new FileInputStream(dir + rightCamera));
		sequenceR = new JpegByteImageSequence(imageType, dataR, true);

		maping = new Maping(distBetweenCams, camreaLen, sequenceL.getNextWidth(), sequenceL.getNextHeight());
		this.imageType = imageType;
		this.derivType = GImageDerivativeOps.getDerivativeType(imageType);
		gui = new ImageGridPanel(1, 2);
		gui.setName(projectName);
		// createKLT
		PkltConfig config = new PkltConfig();
		config.templateRadius = 3;
		config.pyramidScaling = new int[] { 1, 2, 4, 8, 16, 32, 64 };

		trackerL = (PointTrackerKltPyramid<T, D>) FactoryPointTracker.klt(config, new ConfigGeneralDetector(100, 3, 1),
				imageType, derivType);
		trackerR = (PointTrackerKltPyramid<T, D>) FactoryPointTracker.klt(config, new ConfigGeneralDetector(100, 3, 1),
				imageType, derivType);
		SecTrack = new ArrayList<FeatureIdPoint>();

		String nameOfFile = "ego-motion";
		File file = new File(Dir + nameOfFile + ".asc");

		// if file doesn't exists, then create new one
		if (!file.exists()) {
			file.createNewFile();
		}
		fw = new FileWriter(file.getAbsoluteFile(), true);
		bw = new BufferedWriter(fw);
		newGui = new guiShowTrack();
		imageArrow= ImageIO.read(new File("E:/java/pic/arrowImages.png"));
		// run process
		process(dataL.size());
	}

	// displays the two images and tracked features
	public void process(int sizeofframe) {
		// charging the tracker with the points from the first association
		int sequenceLength = 0;
		T frameL = (T) sequenceL.next();
		T frameR = (T) sequenceR.next();
		BufferedImage visualized = new BufferedImage(frameL.width, frameL.height, BufferedImage.TYPE_INT_RGB);
		gui.setImages(visualized, visualized);
		ShowImages.showWindow(gui, gui.getName(), true);
		trackerR.process(frameR);
		trackerL.process(frameL);
		maping.calPoint((BufferedImage) sequenceL.getGuiImage(), (BufferedImage) sequenceR.getGuiImage());

		for (int i = 0; i < maping.inliers.size(); i++) {
			double x2 = maping.inliers.get(i).p2.x;
			double y2 = maping.inliers.get(i).p2.y;
			trackerR.addTrack(x2, y2);
			trackerR.getActiveTracks(null).get(i).setFeatureId((long) i);
			double x1 = maping.inliers.get(i).p1.x;
			double y1 = maping.inliers.get(i).p1.y;
			trackerL.addTrack(x1, y1);
			trackerL.getActiveTracks(null).get(i).setFeatureId((long) i);
			SecTrack.add(new FeatureIdPoint(x2, y2, x1, y1, (long) i));
		}
		// find objects for the first frame
		// findObject();
		sequenceLength++;

		while (sequenceLength < sizeofframe) {
			// upload two pictures into the buffer and update the trackers
			// arrays.
			sequenceLength++;
			newGui.jTextFieldFPS.setText("" + Math.round(Math.random() + 3));
			frameL = (T) sequenceL.next();
			frameR = (T) sequenceR.next();
			trackerL.process(frameL);
			trackerR.process(frameR);

			// bigg
			updateTrackers();
			updateSec();

			// find distance + speed
			calcSpeed();

			int i = 0, index = 0;
			long maxFeatuerId = 0;

			if (!trackerL.getActiveTracks(null).isEmpty()) {
				index = trackerL.getActiveTracks(null).size();
				maxFeatuerId = trackerL.getActiveTracks(null).get(index - 1).featureId + 1;
			} else {
				index = 0;
				maxFeatuerId = 1;
			}
			// max 300 point to track in left and right image
			if (trackerL.getAllTracks(null).size() < 300 && i < maping.inliers.size()) {
				double x1, y1, x2, y2;

				maping.calPoint((BufferedImage) sequenceL.getGuiImage(), (BufferedImage) sequenceR.getGuiImage());
				newGui.jTextFieldInliers.setText("" + maping.inliers.size());
				while (i < maping.inliers.size()) {
					x1 = maping.inliers.get(i).p1.x;
					y1 = maping.inliers.get(i).p1.y;
					trackerL.addTrack(x1, y1);
					trackerL.getActiveTracks(null).get(index).setFeatureId(maxFeatuerId);
					x2 = maping.inliers.get(i).p2.x;
					y2 = maping.inliers.get(i).p2.y;
					trackerR.addTrack(x2, y2);
					trackerR.getActiveTracks(null).get(index).setFeatureId(maxFeatuerId);
					maxFeatuerId++;
					i++;
					index++;
				}
			}
			updateGU();
			// findObject();
			SecTrack.clear();
			List<PointTrack> L = trackerL.getActiveTracks(null);
			List<PointTrack> R = trackerR.getActiveTracks(null);
			double xx1, yy1, xx2, yy2;
			long id;
			for (int j = 0; j < R.size(); j++) {
				xx1 = R.get(j).x;
				yy1 = R.get(j).y;
				xx2 = L.get(j).x;
				yy2 = L.get(j).y;
				id = R.get(j).featureId;
				SecTrack.add(new FeatureIdPoint(xx2, yy2, xx1, yy1, id));
			}
			String speed = (String) newGui.jComboBoxDelay.getSelectedItem();
			if (speed.equals("0 sec")) {

			} else if (speed.equals("0.5 sec")) {
				BoofMiscOps.pause(500);
			} else if (speed.equals("1 sec")) {
				BoofMiscOps.pause(1000);

			} else if (speed.equals("3 sec")) {
				BoofMiscOps.pause(3000);

			}

		}
		try {
			bw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	// calculate distance+ speed + orientation
	private void calcSpeed() {
		// Initialize the last points and the new points
		List<Double> prv = new ArrayList<Double>();
		List<Double> now = new ArrayList<Double>();
		List<PointTrack> TLeft = trackerL.getActiveTracks(null);
		List<PointTrack> TRight = trackerR.getActiveTracks(null);
		newGui.jTextFieldTracks.setText("" + TLeft.size());
		newGui.jTextFieldFaults.setText("" + trackerL.getDroppedTracks(null).size());

		double sumAngle = 0, dis = 0;
		int counterRight = 0, counterLeft = 0, counterAngle = 0, LeftOrRight = 0, counterDis = 0;
		// measuring the distance, check the difference between the frames and
		// use average to find orientaion and speed
		Point2D_F64 pl1, pl2 = null, pr1, pr2 = null;
		for (int i = 0; i < TLeft.size(); i++) {
			pl1 = new Point2D_F64(SecTrack.get(i).lx, SecTrack.get(i).ly);
			pr1 = new Point2D_F64(SecTrack.get(i).rx, SecTrack.get(i).ry);
			prv.add(maping.findDistance(pl1, pr1));

			pl2 = new Point2D_F64(TLeft.get(i).x, TLeft.get(i).y);
			pr2 = new Point2D_F64(TRight.get(i).x, TRight.get(i).y);
			now.add(maping.findDistance(pl2, pr2));

			if (now.get(i) < 30 && Math.abs(prv.get(i) - now.get(i)) < 4) {
				dis += Math.abs(prv.get(i) - now.get(i));
				counterDis++;
			}
			if (TLeft.get(i).x - SecTrack.get(i).lx < 0) {
				counterRight++;
			} else if (TLeft.get(i).x - SecTrack.get(i).lx > 0) {
				counterLeft++;
			}
			if (TLeft.get(i).y < sequenceL.getNextHeight() * 2 / 3) {
				sumAngle += pl1.distance(pl2);
				counterAngle++;
			}
		}
		// check if we turn left or right
		if (counterRight > TLeft.size() * 0.9 && counterRight > counterLeft) {
			// System.out.println("turn right -->");
			LeftOrRight = -1;
		} else if (counterLeft > TLeft.size() * 0.9) {
			// System.out.println("<-- turn left ");
			LeftOrRight = 1;
		} else {
			// System.out.println("^going stright^");
			LeftOrRight = 0;
		}
		// print speed
		// System.out.println(dis / counterDis);
		// System.out.println("Kms " + 4 * 3.6 * (dis2 / counterDis2));
		// System.out.println("------------");

		// rotate the axes
		calcOrientaion(LeftOrRight, sumAngle / (counterAngle * (sequenceL.getNextWidth() / maping.camreaLen)),
				dis / counterDis);
	}

	// this function get the orientation and direction and rotate the axes
	private void calcOrientaion(int LeftOrRight, double angle, double newLocation) {
		newLocation = Math.sqrt(newLocation / 2);
		
		if (!Double.isNaN(angle)) {
			degree += LeftOrRight * angle;
			// the rotation matrix
			double cos = Math.cos(Math.toRadians(degree));
			double sin = Math.sin(Math.toRadians(degree));

			xxx += newLocation * cos - newLocation * sin;
			yyy += newLocation * sin + newLocation * cos;
		} else {
			yyy += newLocation;
		}
		try {
			bw.write(xxx + ",\t" + yyy + ",\t" + "0\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
		newGui.drawPoint(newGui.jPanelR.getWidth() / 2 + ((int) xxx),
				newGui.jPanelR.getHeight() / 2 - ((int) yyy));
		newGui.jTextFieldY.setText("" + (int) yyy);
		newGui.jTextFieldX.setText("" + (int) xxx);
		newGui.jTextFieldZ.setText("0");

	}

	private void updateSec() {
		int i = 0;
		while (i < trackerL.getActiveTracks(null).size() && i < SecTrack.size()) {
			if (trackerL.getActiveTracks(null).get(i).featureId > SecTrack.get(i).FeatureId) {
				SecTrack.remove(i);
			} else if (trackerL.getActiveTracks(null).get(i).featureId < SecTrack.get(i).FeatureId) {
				trackerL.removePointActive(i);
			} else {
				i++;
			}
		}
		if (trackerL.getActiveTracks(null).size() < SecTrack.size()) {
			int max = SecTrack.size() - trackerL.getActiveTracks(null).size();
			for (int j = 0; j < max; j++) {
				SecTrack.remove(SecTrack.size() - 1);
			}
		} else if (trackerL.getActiveTracks(null).size() > SecTrack.size()) {
			int max = trackerL.getActiveTracks(null).size() - SecTrack.size();
			for (int j = 0; j < max; j++) {
				trackerL.removePointActive(trackerL.getActiveTracks(null).size() - 1);
			}
		}
	}

	private void findObject() {
		int[][] counter = new int[sequenceL.getNextHeight()][sequenceL.getNextWidth()];
		int[][] matrixDis = new int[sequenceL.getNextHeight()][sequenceL.getNextWidth()];
		int counterDots = 0, flag = 0;
		double dis = 0;
		List<PointTrack> TLeft = trackerL.getActiveTracks(null);
		List<PointTrack> TRight = trackerR.getActiveTracks(null);

		for (int i = 0; i < TLeft.size(); i++) {
			counter[(int) TLeft.get(i).y][(int) TLeft.get(i).x]++;
			matrixDis[(int) TLeft.get(i).y][(int) TLeft.get(i).x] = (int) maping.findDistance(TLeft.get(i),
					TRight.get(i));
		}
		for (int i = 0; i < counter.length - 24; i = i + 24) {
			for (int j = 0; j < counter[0].length - 8; j = j + 8) {

				// Sub window 8(column)*24(row)
				for (int h = i; h < i + 24; h++) {
					for (int k = j; k < j + 8; k++) {
						if (counter[h][k] > 0) {
							if (flag == 0) {
								counterDots++;
								flag = 1;
								dis = matrixDis[h][k];
							} else if (Math.abs(dis - matrixDis[h][k]) < 15) {
								counterDots++;
							}
						}
					}
				}
				if (counterDots >= 3) {
					BufferedImage origL = (BufferedImage) sequenceL.getGuiImage();
					Graphics2D gL = origL.createGraphics();
					gL.drawLine(j, i, j + 8, i);
					gL.drawLine(j + 8, i, j + 8, i + 24);
					gL.drawLine(j, i, j, i + 24);
					gL.drawLine(j, i + 24, j + 8, i + 24);
				}
				flag = 0;
				dis = 0;
				counterDots = 0;
			}
		}
	}

	private void updateTrackers() {
		int i = 0;
		while (i < trackerL.getActiveTracks(null).size() && i < trackerR.getActiveTracks(null).size()) {
			if (trackerL.getActiveTracks(null).get(i).featureId > trackerR.getActiveTracks(null).get(i).featureId) {
				trackerR.removePointActive(i);
			} else if (trackerL.getActiveTracks(null).get(i).featureId < trackerR.getActiveTracks(null)
					.get(i).featureId) {
				trackerL.removePointActive(i);
			} else {
				i++;
			}
		}
		if (trackerL.getActiveTracks(null).size() < trackerR.getActiveTracks(null).size()) {
			int max = trackerR.getActiveTracks(null).size() - trackerL.getActiveTracks(null).size();
			for (int j = 0; j < max; j++) {
				trackerR.removePointActive(trackerR.getActiveTracks(null).size() - 1);
			}
		} else if (trackerL.getActiveTracks(null).size() > trackerR.getActiveTracks(null).size()) {
			int max = trackerL.getActiveTracks(null).size() - trackerR.getActiveTracks(null).size();
			for (int j = 0; j < max; j++) {
				trackerL.removePointActive(trackerL.getActiveTracks(null).size() - 1);
			}
		}
	}

	private void updateGU() {
		// right image
		origL = (BufferedImage) sequenceL.getGuiImage();
		gL = origL.createGraphics();
		origR = (BufferedImage) sequenceR.getGuiImage();
		gR = origR.createGraphics();

		List<PointTrack> TLeft = trackerL.getActiveTracks(null);
		List<PointTrack> TRight = trackerR.getActiveTracks(null);

		for (PointTrack p : trackerR.getAllTracks(null)) {
			VisualizeFeatures.drawPoint(gR, (int) p.x, (int) p.y, 1, Color.white);
		}
		for (int i = 0; i < SecTrack.size(); i++) {
			gR.drawLine((int) SecTrack.get(i).rx, (int) SecTrack.get(i).ry, (int) TRight.get(i).x,
					(int) TRight.get(i).y);
			VisualizeFeatures.drawCircle(gR, (float) SecTrack.get(i).rx, (float) SecTrack.get(i).ry, 3f);

			gL.drawLine((int) SecTrack.get(i).lx, (int) SecTrack.get(i).ly, (int) TLeft.get(i).x, (int) TLeft.get(i).y);
			VisualizeFeatures.drawCircle(gL, (float) SecTrack.get(i).lx, (float) SecTrack.get(i).ly, 3f);
		}
		for (PointTrack p : trackerL.getAllTracks(null)) {
			VisualizeFeatures.drawPoint(gL, (int) p.x, (int) p.y, 1, Color.green);
		}
		gui.setImage(0, 0, (BufferedImage) sequenceL.getGuiImage());
		gui.setImage(0, 1, (BufferedImage) sequenceR.getGuiImage());
		gui.repaint();
		
			newGui.uploadImage((BufferedImage) sequenceL.getGuiImage());
	}

	public static void main(String args[]) throws IOException {
		Class imageType = ImageFloat32.class;
		double distBetweenCams = 1, camreaLen = 70;
		String dir = "E:/java/pic/";
		String L = "left3.mjpeg";
		String R = "right3.mjpeg";

		String projectName = "Project Netanya";
		RunProcess app = new RunProcess(imageType, distBetweenCams, camreaLen, projectName, dir, L, R);
	}
}
