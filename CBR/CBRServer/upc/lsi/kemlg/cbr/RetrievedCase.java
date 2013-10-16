package upc.lsi.kemlg.cbr;


/**
 * This class defines a special kind of objects used in the Retrieval step.<br>
 * The methods retrieve1Case and retrieveNCases of class CBR use
 * this class to give the results of the retrieval step, that is, one or
 * several clases along with the similarities among this/these cases and the 
 * current case. Similarities are used then to select the best case.
 * <br>
 * A RetrievedCase is an object that is composed by:
 * <ul> <li> the case, obtained from the case library
 *      <li> the similarity ratio among the retrieved case and the current case
 * </ul>
 * @see CBR
 * @see Case
 */
public class RetrievedCase implements Comparable, java.io.Serializable
{      

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** The case retrieved from the Case Lib. */ 
    private Case retrCase;   //Caso obtenido
      
  /** The similarity ratoi to the current case */    
    private float simToCurrent; //similiaridad al caso actual
      


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  

  /**
   * Creates a new retrieved case with the given case and similatiti ratio. <br> 
   * <br>
   * {PRE: retrCase is an instance of Case,
           simToCurrent is a float value, the similarity ratio among
                        retrCase and the current case.
   * }<br>
   * {POST: an instance of RetrievedCase has been created.
   * }<br>
   */
    public RetrievedCase (Case retrCase, float simToCurrent)
    {
       this.retrCase = retrCase;
       this.simToCurrent = simToCurrent;
    }
    

  /**
   * Gets the identifier of the case. <br> 
   * <br>
   * {PRE: x is an instance of RetrievedCase
   * }<br>
   * {POST: returns the identifier of the case inside x.
   * }<br>
   * <br>
   * @return String the identifier of the Case. 
   */
    public String getIdCase()
    {
       return retrCase.getIdCase();
    }
    

  /**
   * Gets the case instance inside the RetrievedCase. <br> 
   * <br>
   * {PRE: x is an instance of RetrievedCase
   * }<br>
   * {POST: returns the case inside x.
   * }<br>
   * <br>
   * @return Case the case inside this RetrievedCase. 
   */
    public Case getCase()
    {
       return retrCase;
    }
    

  /**
   * Gets the similarity ratio among the retrieved case and the current case. <br> 
   * <br>
   * {PRE: x is an instance of RetrievedCase
   * }<br>
   * {POST: returns the similarity ratio among the case inside x 
   *        and the current case.
   * }<br>
   * <br>
   * @return float the similarity ratio. 
   */
    public float getSimToCurrent()
    {
       return simToCurrent;
    }
    
    
  /**
   * Tells if two instances of RetrievedCase are equal. <br>
   * It only compares the name of the case, as a case can
   * only appear once in a retrieved list of cases.
   * {PRE: x is an instance of Modality. 
   *       o is an Object  
   * }<br>
   * {POST: returns true if o is an instance of RetrievedCase
   *        and the name of the case in o is the same as the 
   *        name of the case in x.
   * }<br>
   */     
    public boolean equals(Object o)
    {
       boolean res = false;
       
       if (o instanceof RetrievedCase)
       {
          res = getIdCase().equals(((RetrievedCase) o).getIdCase());
       }
       // else res es false, como ya esta desde que se declaró
       
       return res;
    }

    
  //método de la interficie Comparable

  /**
   * Compares this RetrievedCase to another Object.<br>
   * If the Object is a RetrievedCase, then it makes the comparison of
   * the values of both. Otherwise, an exception is thrown.
   * <b>Note: this method is only useful to do the ordering of the
   *          retrieved cases</b>, from the most similar to the less one.
   * <br>
   * {PRE: x is an instance of RetrievedCase. 
   *       o is an Object
   * }<br>
   * {POST: returns 0 if the similarity ratio of o is equal to the 
   *        similarity ratio of x; a value less than 0 if the 
   *        similarity ratio of o is greater than the similarity ratio
   *        of x; a value greater than 0 if the similarity ratio of
   *        o is lower than the similarity ratio of x.
   * }<br>
   * @param o  the Object to be compared        
   * @return  the signed comparison of the attribute values of x and o. 
   */
    public int compareTo(Object o) 
    {
       float sim;
       int res = 0;
       
       if (o instanceof RetrievedCase)
       {
          sim = (getSimToCurrent() - ((RetrievedCase) o).getSimToCurrent());
          if ((sim > 0.0) && (sim < 1.0))
          {
            res = 1;
          }
          else if ((sim > -1.0) && (sim < 0.0))
          {
            res = -1;
          }   
          else /*(sim >= 1.0) || (sim == 0.0) || (sim <= -1.0)*/
          {
            res = (int) sim; 
          }
          
          if (res == 0)
          {
             res = getIdCase().compareTo(((RetrievedCase) o).getIdCase());
          }   
       
       }
       else
       {
          throw new ClassCastException ("Object passed is not instance of RetrievedCase");  
       }
       
       return res;   
    }
    
    
  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of RetrievedCase 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString()
    {
       return ("[" + getIdCase() + "  sim=" + getSimToCurrent() + "]");
    }
    
}