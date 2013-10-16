package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This abstract class defines a standard to be followed by all
 * description models of a case solution in a CBR. <br>
 * The case solution is usually a model of the solution to a problem.
 * The model lets to describe the solution (an action plan, a set
 * of parameters of a process...) and an evaluation of the soundness
 * of the solution. 
 * The case solution is highly context-dependent, so for each problem
 * a case solution should be done by defining subclasses of this class.
 * @see CBR
 * @see Case
 */
public abstract class CaseSolution implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  
  /** The solution of the case */ 
    private Object solution;
    
  /** The evaluation of the solution */  
    private Object evaluation; 

  

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
    
  
  /**
   * Creates a case solution with no evaluation associed. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of CaseSolution has been created.
   * }<br>
   */
    public CaseSolution(Object sol)
    {
       solution = sol;
       evaluation = null;
    }
     

  /**
   * Gets the solution in this CaseSolution. <br> 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseSolution.
   * }<br>
   * {POST: returns the solution in x.
   * }<br>
   * <br>
   * @return Object the solution in the Case Solution. 
   */
    public Object getSolution()
    {
       return solution;
    }
    

  /**
   * Gets the evaluation of the solution in this CaseSolution. <br> 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseSolution.
   * }<br>
   * {POST: returns the evaluation of the solution in x.
   * }<br>
   * <br>
   * @return Object the evaluation of the solution in the Case Solution. 
   */
    public Object getEvaluation()
    {
       return evaluation;
    }
    

  /**
   * Sets a new solution in this CaseSolution. <br> 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseSolution.
   *       sol the new solution in x
   * }<br>
   * {POST: sol is the new solution in x
   * }<br>
   * <br>
   * @param Object the new solution in the Case Solution. 
   */
    public void setSolution(Object sol)
    {
       solution = sol;
    }


  /**
   * Sets a new evaluation of the solution in this CaseSolution. <br> 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseSolution.
   *       eval the new evaluation of the solution in x
   * }<br>
   * {POST: eval is the new evaluation of the solution in x
   * }<br>
   * <br>
   * @param Object the new evaluation of the solution in the Case Solution. 
   */
    public void setEvaluation(Object eval)
    {
       evaluation = eval;
    }
    


  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of any non-abstract subclass of CaseSolution 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public abstract String toString();

}