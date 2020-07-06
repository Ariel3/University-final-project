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
import boofcv.factory.feature.associate.ConfigAssociateGreedy;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.EnumFundamental;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.TupleDesc_F64;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageGray;
import boofcv.utils.XYZPoint;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.fitting.modelset.ModelManager;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;
import org.ddogleg.struct.FastAccess;
import org.ejml.data.DMatrixRMaj;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * A Fundamental matrix describes the epipolar relationship between two images.
 * If two points, one from each image, match, then the inner product around the
 * Fundamental matrix will be zero. If a fundamental matrix is known, then
 * information about the scene and its structure can be extracted.
 * <p>
 * Below are two examples of how a Fundamental matrix can be computed using
 * different. The robust technique attempts to find the best fit Fundamental
 * matrix to the data while removing noisy matches, The simple version just
 * assumes that all the matches are correct. Similar techniques can be used to
 * fit various other types of motion or structural models to observations.
 * <p>
 * The input image and associated features are displayed in a window. In another
 * window, inlier features from robust model fitting are shown.
 *
 * @author Peter Abeles
 */
public class Maping {
    /*
     * same as MeasuringDistanceTests this class holds the function to measure
     * distance but here it statically it is useful in cases we want measure
     * distance for some point without load all the picture again and again
     *
     */
    double distBetweenCams, camreaLen, imageWidth, imageHeight;
    static double myLocation = 0;
    List<AssociatedPair> inliers, matches;

    // variables of robustFundamental function
    // used to create and copy new instances of the fit model
    ModelManager<DMatrixRMaj> managerF;
    // Select which linear algorithm is to be used. Try playing with the
    // number of remove ambiguity points
    Estimate1ofEpipolar estimateF;
    // Wrapper so that this estimator can be used by the robust estimator
    GenerateEpipolarMatrix generateF;
    // How the error is measured
    DistanceFromModelResidual<DMatrixRMaj, AssociatedPair> errorMetric = new DistanceFromModelResidual<DMatrixRMaj, AssociatedPair>(
            new FundamentalResidualSampson());
    // Use RANSAC to estimate the Fundamental matrix
    static ModelMatcher<DMatrixRMaj, AssociatedPair> robustF;

    // variables of computeMatches function
    DetectDescribePoint detDesc;
    ScoreAssociation<TupleDesc_F64> scorer;
    static AssociateDescription<TupleDesc_F64> associate;
    static ExampleAssociatePoints<GrayF32, TupleDesc_F64> findMatches;

    public Maping(double distBetweenCams, double camreaLen, double width, double height) {
        this.distBetweenCams = distBetweenCams;
        this.camreaLen = camreaLen;
        inliers = new ArrayList<AssociatedPair>();
        imageWidth = width;
        imageHeight = height;
        // Preparation function robustFundamental
        managerF = new ModelManagerEpipolarMatrix();
        estimateF = FactoryMultiView.fundamental_1(EnumFundamental.LINEAR_7, 2);
        generateF = new GenerateEpipolarMatrix(estimateF);
        robustF = new Ransac<DMatrixRMaj, AssociatedPair>(123123, managerF, generateF, errorMetric, 6000, 0.1);

        // Preparation function computeMatches
        detDesc = FactoryDetectDescribe.surfStable(new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null, null,
                ImageGray.class);
        scorer = FactoryAssociation.scoreEuclidean(TupleDesc_F64.class, true);
        associate = FactoryAssociation.greedy(new ConfigAssociateGreedy(true), scorer);
        findMatches = new ExampleAssociatePoints<GrayF32, TupleDesc_F64>(detDesc, associate, GrayF32.class);
    }

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

    /**
     * Use the associate point feature example to create a list of
     * {@link AssociatedPair} for use in computing the fundamental matrix.
     */
    public void computeMatches(BufferedImage left, BufferedImage right) {

        findMatches.associate(left, right);

        List<AssociatedPair> matches = new ArrayList<AssociatedPair>();
        FastAccess<AssociatedIndex> matchIndexes = associate.getMatches();
        AssociatedIndex a;
        AssociatedPair p;
        for (int i = 0; i < matchIndexes.size; i++) {
            a = matchIndexes.get(i);
            p = new AssociatedPair(findMatches.pointsA.get(a.src), findMatches.pointsB.get(a.dst));
            matches.add(p);
        }
        // Estimate the fundamental matrix while removing outliers
        if (!robustF.process(matches))
            throw new IllegalArgumentException("Failed");

        // save the set of features that were used to compute the fundamental
        // matrix
        inliers.addAll(robustF.getMatchSet());

    }

    // this function create all the lists and call for all the helper functions.
    public void calPoint(BufferedImage Left, BufferedImage Right) {

        inliers.clear();
        computeMatches(Left, Right);

        List<XYZPoint> XYZ = new ArrayList<XYZPoint>();
        for (int i = 0; i < inliers.size(); i++) {
            XYZPoint p = findXYZ(inliers.get(i).p1, inliers.get(i).p2);
            if (p.getX() < 200 && p.getY() < 200 && p.getZ() < 200) {
                XYZ.add(p);
            }
        }
    }

    // this function measure distance and return the coordinate where the
    // distance end
    public XYZPoint findXYZ(Point2D_F64 pLeft, Point2D_F64 pRight) {
        double y, real_len_of_pic_axe_x, meter_per_pixel_x, x, meter_per_pixel_y, z;
        y = (imageWidth * distBetweenCams) / ((2 * Math.toRadians(camreaLen / 2)) * (pLeft.x - pRight.x));
        real_len_of_pic_axe_x = Math.toRadians(camreaLen / 2) * 2 * y;
        meter_per_pixel_x = real_len_of_pic_axe_x / imageWidth;
        x = (pLeft.x - (imageWidth / 2)) * meter_per_pixel_x + distBetweenCams / 2;
        meter_per_pixel_y = real_len_of_pic_axe_x / imageHeight;
        z = (imageHeight / 2) - pLeft.y;
        z = z * meter_per_pixel_y;
        XYZPoint ans = new XYZPoint(x, y, z);

        return ans;
    }

    // this function measure distance
    public double findDistance(Point2D_F64 pLeft, Point2D_F64 pRight) {
        double y, real_len_of_pic_axe_x, meter_per_pixel_x, x, dis, meter_per_pixel_y, z, real_dis;

        y = (imageWidth * distBetweenCams) / ((2 * Math.toRadians(camreaLen / 2)) * (pLeft.x - pRight.x));
        real_len_of_pic_axe_x = Math.toRadians(camreaLen / 2) * 2 * y;
        meter_per_pixel_x = real_len_of_pic_axe_x / imageWidth;
        x = (pLeft.x - (imageWidth / 2)) * meter_per_pixel_x + distBetweenCams / 2;
        dis = Math.sqrt(x * x + y * y);
        meter_per_pixel_y = real_len_of_pic_axe_x / imageHeight;

        z = (imageHeight / 2) - pLeft.y;
        z = z * meter_per_pixel_y;
        real_dis = Math.sqrt(z * z + dis * dis);

        return real_dis;
    }
}