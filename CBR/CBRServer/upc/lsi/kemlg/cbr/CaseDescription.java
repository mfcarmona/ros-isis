package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This abstract class defines a standard to be followed by all
 * description models of a case in a CBR. <br>
 * The case description is usually a model of the problem to solve.
 * The model lets to compare descriptions with a similarity measure
 * in order to find the past experience (case) similar to the 
 * current one. 
 * @see CBR
 * @see Case
 * @see SimilarityMeasure
 */
public abstract class CaseDescription implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

 
  /** Similarity measure used to compare descriptions */
    SimilarityMeasure simMeasure;
    
    
    
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
    
  
  /**
   * Creates an empty case description associed to the given similarity measure. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of CaseDescription has been created.
   * }<br>
   */
    public CaseDescription(SimilarityMeasure sim)
    {
       simMeasure = sim;
    }
     

  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  


  /**
   * Compares this case description with a given one. <br>
   * The comparison is made on a similarity basis
   * (SimilarityMeasure).
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseDescription,
   *       c2 is an instance of any non-abstract subclass of CaseDescription
   * }<br>
   * {POST: returns a similarity ratio among x and c2.
   * }<br>
   * @param c2 the other case description to compare with. 
   * @return float the similatity ratio among this case description and c2 
   */
    public abstract float similarTo(CaseDescription c2);


  /**
   * Gets the similarity measure. <br> 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseDescription
   * }<br>
   * {POST: returns the similarity measure associed to x.
   * }<br>
   * <br>
   * @return SimilarityMeasure the similarity measure associed to this CaseDescription. 
   */
    public abstract SimilarityMeasure getSimilarityMeasure();


  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseDescription 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public abstract String toString();
}