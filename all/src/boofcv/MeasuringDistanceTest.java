/*
 * Copyright (c) 2011-2015, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.geo.Estimate1ofEpipolar;
import boofcv.abst.geo.fitting.DistanceFromModelResidual;
import boofcv.abst.geo.fitting.GenerateEpipolarMatrix;
import boofcv.abst.geo.fitting.ModelManagerEpipolarMatrix;
import boofcv.alg.geo.f.FundamentalResidualSampson;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.EnumEpipolar;
import boofcv.factory.geo.EpipolarError;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.SurfFeature;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.fitting.modelset.ModelFitter;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;
import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * A Fundamental matrix describes the epipolar relationship between two images.
 * If two points, one from each image, match, then the inner product around the
 * Fundamental matrix will be zero. If a fundamental matrix is known, then
 * information about the scene and its structure can be extracted.
 *
 * Below are two examples of how a Fundamental matrix can be computed using
 * different. The robust technique attempts to find the best fit Fundamental
 * matrix to the data while removing noisy matches, The simple version just
 * assumes that all the matches are correct. Similar techniques can be used to
 * fit various other types of motion or structural models to observations.
 *
 * The input image and associated features are displayed in a window. In another
 * window, inlier features from robust model fitting are shown.
 *
 * @author Peter Abeles
 */

/*
 * in this class we want to measure distance by two photos we will find matches
 * between the pictures and by geometric manipulations measure the distance.
 * input: directory with the two pictures output: list with all the coordinates
 * and the distance between them.
 */

public class MeasuringDistanceTest {
	static double index = 0;

	/**
	 * Given a set of noisy observations, compute the Fundamental matrix while
	 * removing the noise.
	 *
	 * @param matches
	 *            List of associated features between the two images
	 * @param inliers
	 *            List of feature pairs that were determined to not be noise.
	 * @return The found fundamental matrix.
	 */
	public static DenseMatrix64F robustFundamental(List<AssociatedPair> matches, List<AssociatedPair> inliers) {

		// used to create and copy new instances of the fit model
		ModelManager<DenseMatrix64F> managerF = new ModelManagerEpipolarMatrix();
		// Select which linear algorithm is to be used. Try playing with the
		// number of remove ambiguity points
		Estimate1ofEpipolar estimateF = FactoryMultiView.computeFundamental_1(EnumEpipolar.FUNDAMENTAL_7_LINEAR, 2);
		// Wrapper so that this estimator can be used by the robust estimator
		GenerateEpipolarMatrix generateF = new GenerateEpipolarMatrix(estimateF);

		// How the error is measured
		DistanceFromModelResidual<DenseMatrix64F, AssociatedPair> errorMetric = new DistanceFromModelResidual<DenseMatrix64F, AssociatedPair>(
				new FundamentalResidualSampson());

		// Use RANSAC to estimate the Fundamental matrix
		ModelMatcher<DenseMatrix64F, AssociatedPair> robustF = new Ransac<DenseMatrix64F, AssociatedPair>(123123,
				managerF, generateF, errorMetric, 6000, 0.1);

		// Estimate the fundamental matrix while removing outliers
		if (!robustF.process(matches))
			throw new IllegalArgumentException("Failed");

		// save the set of features that were used to compute the fundamental
		// matrix
		inliers.addAll(robustF.getMatchSet());

		// Improve the estimate of the fundamental matrix using non-linear
		// optimization
		DenseMatrix64F F = new DenseMatrix64F(3, 3);
		ModelFitter<DenseMatrix64F, AssociatedPair> refine = FactoryMultiView.refineFundamental(1e-8, 400,
				EpipolarError.SAMPSON);
		if (!refine.fitModel(inliers, robustF.getModelParameters(), F))
			throw new IllegalArgumentException("Failed");

		// Return the solution
		return F;
	}

	/**
	 * If the set of associated features are known to be correct, then the
	 * fundamental matrix can be computed directly with a lot less code. The
	 * down side is that this technique is very sensitive to noise.
	 */
	public static DenseMatrix64F simpleFundamental(List<AssociatedPair> matches) {
		// Use the 8-point algorithm since it will work with an arbitrary number
		// of points
		Estimate1ofEpipolar estimateF = FactoryMultiView.computeFundamental_1(EnumEpipolar.FUNDAMENTAL_8_LINEAR, 0);

		DenseMatrix64F F = new DenseMatrix64F(3, 3);
		if (!estimateF.process(matches, F))
			throw new IllegalArgumentException("Failed");

		// while not done here, this initial linear estimate can be refined
		// using non-linear optimization
		// as was done above.
		return F;
	}

	/**
	 * Use the associate point feature example to create a list of
	 * {@link AssociatedPair} for use in computing the fundamental matrix.
	 */
	public static List<AssociatedPair> computeMatches(BufferedImage left, BufferedImage right) {
		DetectDescribePoint detDesc = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4),
				null, null, ImageFloat32.class);
		// DetectDescribePoint detDesc = FactoryDetectDescribe.sift(null,new
		// ConfigSiftDetector(2,0,200,5),null,null);

		ScoreAssociation<SurfFeature> scorer = FactoryAssociation.scoreEuclidean(SurfFeature.class, true);
		AssociateDescription<SurfFeature> associate = FactoryAssociation.greedy(scorer, 1, true);

		ExampleAssociatePoints<ImageFloat32, SurfFeature> findMatches = new ExampleAssociatePoints<ImageFloat32, SurfFeature>(
				detDesc, associate, ImageFloat32.class);

		findMatches.associate(left, right);

		List<AssociatedPair> matches = new ArrayList<AssociatedPair>();
		FastQueue<AssociatedIndex> matchIndexes = associate.getMatches();

		for (int i = 0; i < matchIndexes.size; i++) {
			AssociatedIndex a = matchIndexes.get(i);
			AssociatedPair p = new AssociatedPair(findMatches.pointsA.get(a.src), findMatches.pointsB.get(a.dst));
			matches.add(p);
		}

		return matches;
	}

	// this function create all the lists and call for all the helper functions.
	public static void calPoint(String pathLeft, String pathRight, double angle, double dist_between_cams) {

		BufferedImage imageA = UtilImageIO.loadImage(pathLeft);
		BufferedImage imageB = UtilImageIO.loadImage(pathRight);
		if (imageA != null && imageB != null) {

			List<AssociatedPair> matches = computeMatches(imageA, imageB);
			List<AssociatedPair> inliers = new ArrayList<AssociatedPair>();
			robustFundamental(matches, inliers);
			List<AssociatedPair> Small_inliers = new ArrayList<AssociatedPair>();
			/*
			 * find only one point by coordinate for (int i = 0; i <
			 * inliers.size(); i++) { if (inliers.get(i).p1.x > 480 &&
			 * inliers.get(i).p1.x < 499 && inliers.get(i).p1.y < 240 &&
			 * inliers.get(i).p1.y > 150) { Small_inliers.add(inliers.get(i));
			 * System.out.println(i); } }
			 */

			List<XYZPoint> XYZ = new ArrayList<XYZPoint>();

			for (int i = 0; i < inliers.size(); i++) {

				XYZPoint p = findXYZ(imageA.getWidth(), imageA.getHeight(), angle, dist_between_cams, inliers.get(i).p1,
						inliers.get(i).p2);
				System.out.println("dist=" + findDistance(imageA.getWidth(), imageA.getHeight(), angle,
						dist_between_cams, inliers.get(i).p1, inliers.get(i).p2));
				p.setY(p.getY());
				XYZ.add(p);

			}

			index = index + 1;
			XYZPoint.writeXYZFormat(XYZ, "C:/Users/Shira/Desktop/Netanel/Project boaz/", "building1884");
			/*
			 AssociationPanel panel1 = new AssociationPanel(20);
			 panel1.setAssociation(inliers);
			 panel1.setImages(imageA, imageB);
			 ShowImages.showWindow(panel1, "inliers Pairs");
			 * 
			 * 
			 * 
			 */
		}
	}
    // this function measure distance and return the coordinate where the distance end 
	public static XYZPoint findXYZ(double width, double height, double angle, double dist_between_cams,
			Point2D_F64 pLeft, Point2D_F64 pRight) {
		double y, real_len_of_pic_axe_x, meter_per_pixel_x, x, meter_per_pixel_y, z;

		y = (width * dist_between_cams) / ((2 * Math.toRadians(angle / 2)) * (pLeft.x - pRight.x));
		real_len_of_pic_axe_x = Math.toRadians(angle / 2) * 2 * y;
		meter_per_pixel_x = real_len_of_pic_axe_x / width;
		x = (pLeft.x - (width / 2)) * meter_per_pixel_x + dist_between_cams / 2;
		meter_per_pixel_y = real_len_of_pic_axe_x / height;

		z = (height / 2) - pLeft.y;
		z = z * meter_per_pixel_y;
		XYZPoint ans = new XYZPoint(x, y, z);
		
		return ans;

	}
	 // this function measure distance
	public static double findDistance(double width, double height, double angle, double dist_between_cams,
			Point2D_F64 pLeft, Point2D_F64 pRight) {
		double y, real_len_of_pic_axe_x, meter_per_pixel_x, x, dis, meter_per_pixel_y, z, real_dis;

		y = (width * dist_between_cams) / ((2 * Math.toRadians(angle / 2)) * (pLeft.x - pRight.x));
		real_len_of_pic_axe_x = Math.toRadians(angle / 2) * 2 * y;
		meter_per_pixel_x = real_len_of_pic_axe_x / width;
		x = (pLeft.x - (width / 2)) * meter_per_pixel_x + dist_between_cams / 2;
		dis = Math.sqrt(x * x + y * y);
		meter_per_pixel_y = real_len_of_pic_axe_x / height;

		z = (height / 2) - pLeft.y;
		z = z * meter_per_pixel_y;
		real_dis = Math.sqrt(z * z + dis * dis);
		return real_dis;

	}

	public static void main(String args[]) {
		/*
		 * load the photos into string configure the angle of the cameras
		 * configure the distance between the cameras. start the procedure.
		 */
		String Left = "C:/Users/Shira/Desktop/Netanel/Project boaz/ZED test/1mateerL.png";
		String Right = "C:/Users/Shira/Desktop/Netanel/Project boaz/ZED test/1mateerR.png";
		double angle = 110, dist_between_cams = 0.12;

		calPoint(Left, Right, angle, dist_between_cams);

	}

}
