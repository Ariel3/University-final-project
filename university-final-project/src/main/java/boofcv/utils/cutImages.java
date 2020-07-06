package boofcv.utils;

import boofcv.io.image.UtilImageIO;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class cutImages {

    public static void main(String[] args) {
        // TODO Auto-generated method stub
        String path = "E:/java/pic/ZED/original/circal";
        String suffix = ".png";
        int width = 1280, height = 720;

        File directory = new File(path);
        File[] files = directory.listFiles();
        List<String> list = new ArrayList<String>();

        for (File f : files) {
            if (f.isFile() && f.getName().endsWith(suffix))
                list.add(f.getPath());
        }
        Collections.sort(list);
        int i = 10;
        for (String n : list) {
            BufferedImage imageA = UtilImageIO.loadImage(n);
            BufferedImage L = imageA.getSubimage(0, 0, width, height);
            BufferedImage R = imageA.getSubimage(width, 0, width, height);
            // retrieve image
            File outputfileL = new File(path + "/" + i + "L.jpg");
            File outputfileR = new File(path + "/" + i + "R.jpg");
            i++;
            try {
                ImageIO.write(L, "jpg", outputfileL);
                ImageIO.write(R, "jpg", outputfileR);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }
}
