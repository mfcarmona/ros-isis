package uma.dte.etools.cbr.distances;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This abstract class defines the Tanimoto distance measure.
 * <br>
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface.</b>
 * @see ComposedDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public class TanimotoDistance extends upc.lsi.kemlg.cbr.distances.ComposedDistance
					implements java.io.Serializable
{


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /**
   * Creates a Tanimoto Distance. <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of TanimotoDistance has been created.
   * }<br>
   */
    public TanimotoDistance()
    {
       super();
    }


  /**
   * Computes the Tanimoto Distance among two distance collections. <br>
   * <br>
   * {PRE: x is an instance of TanimotoDistance,
   *	   d1 is an instance of DistanceCollection and
   *	   d2 is an instance of DistanceCollection
   * }<br>
   * {POST: returns a distance among d1 and d2.
   * }<br>
   * @param d1 a DistanceCollection to be compared.
   * @param d2 a DistanceCollection to be compared.
   * @return float the distance among both distance collections.
   */
    public float computeDistance(DistanceCollection d1,
				 DistanceCollection d2)
    {
       int size_d1, size_d2;
       java.util.Iterator it_d1, it_d2;
       DistanceComparable elem_1, elem_2;
       float res = 0;
       float norm_x = 0;
       float norm_y = 0;
       float tani = 0;

       //comprobación de que tienen la misma cantidad de items
       if (d1.size()==d2.size())
       {
	  it_d1=d1.iterator();
	  it_d2=d2.iterator();


	  while(it_d1.hasNext())
	  {
	     elem_1 = (DistanceComparable) it_d1.next();
	     elem_2 = (DistanceComparable) it_d2.next();

	     norm_x = norm_x + (float) (Math.pow(((LinearAttribute) elem_1).getQuantVal(),2));
	     norm_y = norm_y + (float) (Math.pow(((LinearAttribute) elem_2).getQuantVal(),2));

			 tani = tani + (((LinearAttribute) elem_1).getQuantVal()*((LinearAttribute) elem_2).getQuantVal());
	  }

	  res = tani / (norm_x + norm_y - tani);
       }
       else
       {
	  System.out.println("error!!!!!! Comparing arrays of different sizes.");
       }

       return res;
    }

}
