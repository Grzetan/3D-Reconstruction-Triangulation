[Wczoraj 12:55] Adam Świtoński
/*
* Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
* Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
*/
package aswitonski.hml; /**
*
* @author Piotrek
*/
/*
* Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
* Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
*/
import java.util.List;
import static java.lang.Math.sqrt;
import static java.lang.Math.abs;
import java.util.ArrayList;
import org.opencv.core.Mat; import org.opencv.core.Point3;  public class PHasiec {     private Point3 versorX;
    private Point3 versorY;
    private Point3 versorZ;
    public static Point3 sum(Point3 p1, Point3 p2) {
        return new Point3(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
    }     public static Point3 mul(Point3 p, double scaler) {
        return new Point3(p.x * scaler, p.y * scaler, p.z * scaler);
    }
    public Point3[] boundingCuboid(Point3[] cross, Point3[] localCuboid) {         if (cross.length != 5 || localCuboid.length != 8) {
            return null;
        }
        this.versorX = sum(cross[1], mul(cross[0],-1));//obliczam wektor kierunkowy osi x
        versorX = mul(versorX, 1 / Math.sqrt(versorX.dot(versorX)));
        // normalizacja 
        this.versorY = sum(cross[3], mul(cross[2], -1));//obliczam wektor kierunkowy osi y
        versorY = mul(versorY, 1 / Math.sqrt(versorY.dot(versorY)));// normalizacja         this.versorZ = versorX.cross(versorY);//obliczam wersor kierunkowy osi z jako ilczyn wektorowy wersorów x i y - będzie porostopadły
        //versorZ = versorZ / sqrt(versorZ.dot(versorZ));         Point3[] globalCuboid = new Point3[localCuboid.length];
        for (int i = 0; i < localCuboid.length; i++) {
            Point3 p = localCuboid[i];
            globalCuboid[i] = sum(cross[4], sum(mul(versorX, p.x), sum(mul(versorY, p.y), mul(versorZ, p.z))));         }
        return globalCuboid;
    }     /**
     * Funkcja oblicza punkt przecięcia dla ramion krzyża w 3d z uwzględnieniem,
     * że jeden z 4 punktów może nie być współpaszczyznowy z pozostałymi trzema
     *
     * @param[in] four_points wektor 4 punktów ramion krzyża w postaci
     * niekoniecznie uporządkowanej
     * @param[in] arms wektor dwóch wartości stanowiących długości ramion krzyża
     * @param[in] err maksymalny dopuszczalny błąd między faktyczną długością
     * ramienia, a długością obliczoną na podstawie punktów (w jednostkach tych
     * samych co pozycje markerów ramion)
     * @return wektor 5 punktów krzyża w postaci [1. punkt ramienia x,2. punkt
     * ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt przecięcia]
     */
    public static Point3[] cross3d(Point3[] four_points, double[] arms, double err) {
        //znalezienie które ramiona znajdują sie naprzeciwko siebie poprzez sprawdzenie odległości między każdym możliwym dopasowaniem punktów w pary
        int[] arms_order=null;         for (int[] j : new int[][] {{1,2,3}, {2,1,3},{3,1,2}})
        {            
            Point3 arm1 = sum(four_points[0],mul(four_points[j[0]],-1));
            Point3 arm2 = sum(four_points[j[1]],mul(four_points[j[2]],-1));
            if ((Math.abs(Math.sqrt(arm1.dot(arm1))- arms[0]) < err && abs(sqrt(arm2.dot(arm2)) - arms[1]) < err) || (abs(sqrt(arm1.dot(arm1)) - arms[1]) < err && abs(sqrt(arm2.dot(arm2)) - arms[0]) < err)) {
                arms_order = j;
                break;
            }
        }         if (arms_order==null) {
            return null;
        }
        //poniższe obliczenia zostały wyprowadzone z warunku, że iloczyn skalarny między wektorem rozpiętym od punktu środkowego do jednego z ramion, a wektorem od punktu środkowego do końca ramienia prostopadłego powinien wynosić 0
        Point3 delta_collinear = sum(four_points[arms_order[0]],mul(four_points[0],-1));
        Point3 delta_non_collinear = sum(four_points[arms_order[1]],mul(four_points[0],-1));
        double denominator = (delta_collinear.x * delta_collinear.x + delta_collinear.y * delta_collinear.y + delta_collinear.z * delta_collinear.z);
        double t = (delta_collinear.x * delta_non_collinear.x + delta_collinear.y * delta_non_collinear.y + delta_collinear.z * delta_non_collinear.z) / denominator;
        Point3 cross_point = sum(four_points[0],mul(delta_collinear, t));
        return new Point3[] {four_points[0], four_points[arms_order[0]],
            four_points[arms_order[1]], four_points[arms_order[2]], cross_point};       
    }
    public  static Point3[] cross3d(Point3[] four_points) {
        return cross3d(four_points, new double[] {180,80}, 120);
    }
    /**
     * A method written from the conversion between quaternions and rotation matrix
     * @return rotation quaternion of local system
     * @throws Exception
     */
    public ArrayList<Double> rotation_quaternion() throws Exception
    {         if(this.versorX== null ||this.versorY== null||this.versorZ== null )
        {
            throw new Exception("One or more axis versor are null");         }
        ArrayList<Double> rotation_quat = new ArrayList<>();
        Double w = Math.sqrt(1+this.versorX.x + this.versorY.y +this.versorZ.z)/2;
        Double qx = (this.versorZ.y - this.versorY.z)/(4*w);
        Double qy = (this.versorZ.x - this.versorX.z)/(4*w);
        Double qz = (this.versorX.y - this.versorY.x)/(4*w);
        rotation_quat.add(w);
        rotation_quat.add(qx);
        rotation_quat.add(qy);
        rotation_quat.add(qz);
        return rotation_quat;
    }
    /**
      *A method written on the basis of the geometric interpretation of the quatrmion rotation.
     * @return rotation quaternion of local system
     * @throws Exception
     */
    public ArrayList<Double> rotation_quaternion2() throws Exception
    {         if(this.versorX== null ||this.versorY== null||this.versorZ== null )
        {
            throw new Exception("One or more axis versor are null");         }
        ArrayList<Double> rotation_quat = new ArrayList<>();
        var vx = new Point3(1,0,0);
        var vy = new Point3(0,1,0);
        Point3 d1 = sum( mul(this.versorX,-1),vx);
        Point3 d2 = sum( mul(this.versorY,-1),vy);
        Double m = Math.sqrt( Math.pow(d1.z*d2.y - d1.y*d2.z,2) +Math.pow(d1.y*d2.x - d2.y*d1.x,2) +Math.pow(d2.z*d1.x - d1.z*d2.x,2)  );
        double sign = Math.signum(d1.y*d2.x-d1.x*d2.y);
        Double qx = sign*(d1.y*d2.z-d1.z*d2.y)/m;
        Double qy = sign*(d1.x*d2.z-d1.z*d2.x)/m;
        Double qz = Math.abs(d1.y*d2.x-d1.x*d2.y)/m;
        Double t = (this.versorX.x*qx + this.versorX.y*qy+this.versorX.z*qz)/(Math.pow(qx,2) +Math.pow(qy,2)  +Math.pow(qz,2)); 
        Point3 vt = mul(new Point3(qx,qy,qz),-t);
        Point3 v1 = sum(this.versorX,vt);
        Point3 v2 = sum(vx,vt);
        Double cos_a = (v1.dot(v2))/Math.sqrt( v1.dot(v1) * v2.dot(v2));
        rotation_quat.add(Math.pow((cos_a+1)/2,0.5) );
        rotation_quat.add( qx*Math.pow((1-cos_a)/2,0.5));
        rotation_quat.add(qy*Math.pow((1-cos_a)/2,0.5));
        rotation_quat.add(qz*Math.pow((1-cos_a)/2,0.5));
        return rotation_quat;
    }
    public Point3[] compute_versor(Point3[] cross)
    {
        this.versorX = sum(cross[1], mul(cross[0],-1));//obliczam wektor kierunkowy osi x
        versorX = mul(versorX, 1 / Math.sqrt(versorX.dot(versorX)));
        // normalizacja 
        this.versorY = sum(cross[3], mul(cross[2], -1));//obliczam wektor kierunkowy osi y
        versorY = mul(versorY, 1 / Math.sqrt(versorY.dot(versorY)));// normalizacja         this.versorZ = versorX.cross(versorY);//obliczam wersor kierunkowy osi z jako ilczyn wektorowy wersorów x i y - będzie porostopadły
        //versorZ = versorZ / sqrt(versorZ.dot(versorZ));
        return new Point3[] {this.versorX,this.versorY,this.versorZ};
    }
    /**
     *
     * @return Matrix of rotation
     * @throws Exception
     */
    public Double[][] rotation_matrix() throws Exception
    {
        if(this.versorX== null ||this.versorY== null||this.versorZ== null )
        {
            throw new Exception("One or more axis versor are null");         }
        Double[][] rotation_matrix = new Double[][]{{versorX.x,versorY.x,versorZ.x},{versorX.y,versorY.y,versorZ.y},{versorX.z,versorY.z,versorZ.z}};         
        return rotation_matrix;
    }
}

