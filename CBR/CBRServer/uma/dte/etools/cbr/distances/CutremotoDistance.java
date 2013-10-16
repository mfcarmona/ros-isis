package uma.dte.etools.cbr.distances;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This abstract class defines the Cutremoto distance measure.
 * <br>
 * <b>Note: This distance can be applied only to collections that
 * implement the DistanceCollection interface.</b>
 * @see ComposedDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public class CutremotoDistance extends upc.lsi.kemlg.cbr.distances.ComposedDistance
					implements java.io.Serializable
{

    float k;  // constant value


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================


  /**
   * Creates a Cutremoto Distance. <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of CutremotoDistance has been created.
   * }<br>
   */
    public CutremotoDistance(float k)
    {
       super();
	   this.k = k;
    }


  /**
   * Computes the Cutremoto Distance among two distance collections. <br>
   * <br>
   * {PRE: x is an instance of CutremotoDistance,
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
       float res = 0.0f;
	   double d_res = 0.0;
	   double d_k = this.k;

       //comprobación de que tienen la misma cantidad de items
       if (d1.size()==d2.size())
       {
	  it_d1=d1.iterator();
	  it_d2=d2.iterator();


	  while(it_d1.hasNext())
	  {
	     elem_1 = (DistanceComparable) it_d1.next();
	     elem_2 = (DistanceComparable) it_d2.next();

	     d_res = d_res + Math.exp(- d_k * (double)distanceTwoItems(elem_1, elem_2));
	  }

	  res = (float)d_res / (float)d1.size();
       }
       else
       {
	  System.out.println("error!!!!!! Comparing arrays of different sizes.");
       }

       return res;
    }


    /**
     * Computes the distance among two elements of different distance collections. <br>
     * <br>
     * {PRE: x is an instance of a non-abstract subclass of MinkowskyDistance,
     *	     d1 is an instance of DistanceComparable and
     *	     d2 is an instance of DistanceComparable
     * }<br>
     * {POST: returns a distance among d1 and d2.
     * }<br>
     * @param d1 a DistanceComparable to be compared.
     * @param d2 a DistanceComparable to be compared.
     * @return float the distance among both elements.
     */
      protected float distanceTwoItems (DistanceComparable d1,
					DistanceComparable d2)
      {
		 return Math.abs(((LinearAttribute) d1).getQuantVal() - ((LinearAttribute) d2).getQuantVal());
      }

}
