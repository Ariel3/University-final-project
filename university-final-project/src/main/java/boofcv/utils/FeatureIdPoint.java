package boofcv.utils;

/*
this class holds the id of the feature for each point (in pixel) 
it is useful for the tracker.
by the id the tracker will check each time if it still has the point with id or if to throw the points.  
 */
public class FeatureIdPoint {
    public double lx, ly, rx, ry;
    public long featureId;

    //Constructor
    public FeatureIdPoint(double lx, double ly, double rx, double ry, long featureId) {
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
        this.featureId = featureId;

    }
}
