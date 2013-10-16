package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This class defines a case, that is, an element of the Case Library of a Case Base Reasoner (CBR). <br>
 * <br>
 * The Case Library is the part of the CBR that stores a set of cases
 * that represent the collected experience of a given domain (for 
 * instance, a set of past happenings and the solutions that were
 * taken).
 * <br>
 * A case is composed of two parts:
 * <UL> <LI> a case description, useful for the retrieve step. 
 *      <LI> a case solution, provided as result of the retrieval.
 * </UL> 
 * @see CBR
 * @see CaseDescription
 * @see CaseSolution
 */
public class Case implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================
 
  /** The identifier of the case in the Case Library. */
    private String idCase=null; //idCase asignado por CBR

  /** The case description. */
    private CaseDescription desc; //descripción del caso (incluida medida de
                                  //similaridad) 

  /** The case solution. */
    private CaseSolution sol; //solución (y evaluación) a ese caso


    
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================

  
  /**
   * Creates a new case with the given case description and case solution. <br> 
   * <br>
   * {PRE: desc is an instance of CaseDescription,
           sol is an instance of CaseSolution
   * }<br>
   * {POST: an instance of Case has been created.
   * }<br>
   */
    public Case(CaseDescription desc, CaseSolution sol)
    {
       this.desc = desc;
       this.sol = sol;
    }
    

  /**
   * Gets the identifier of the case. <br> 
   * This identifier is usually created automatically by the CBR.
   * <br>
   * {PRE: x is an instance of Case
   * }<br>
   * {POST: returns the identifier of x.
   * }<br>
   * <br>
   * @return String the identifier of the Case. 
   */
    public String getIdCase()
    { 
       return idCase;
    }
    

  /**
   * Sets the identifier of the case. <br> 
   * This method is usually invoked by the CBR, which automatically
   * creates unique indentifiers for all the cases in the case library.
   * <br>
   * {PRE: x is an instance of Case
   *       id is a String value suitable to identify the case
   * }<br>
   * {POST: id is the identifier of x.
   * }<br>
   * <br>
   * @param id the identifier of the Case. 
   */
    public void setIdCase(String id)
    {
      idCase = id;
    } 
    

  /**
   * Gets the case description. <br> 
   * <br>
   * {PRE: x is an instance of Case
   * }<br>
   * {POST: returns the case description of x.
   * }<br>
   * <br>
   * @return CaseDescription the description of the Case. 
   */
    public CaseDescription getCaseDescription()
    {
       return desc;
    }  
    

  /**
   * Sets a new case description. <br> 
   * This method is useful to change the initial description of a case.
   * <br>
   * {PRE: x is an instance of Case
   *       desc is an instance of CaseDescription
   * }<br>
   * {POST: desc is the new case description of x.
   * }<br>
   * <br>
   * @param CaseDescription the new case description. 
   */
    public void setCaseDescription(CaseDescription desc)
    {
       this.desc = desc;
    }  


  /**
   * Gets the case solution. <br> 
   * <br>
   * {PRE: x is an instance of Case
   * }<br>
   * {POST: returns the case solution of x.
   * }<br>
   * <br>
   * @return CaseSolution the solution of the Case. 
   */
    public CaseSolution getCaseSolution()
    {
       return sol;
    }  


  /**
   * Sets a new case solution. <br> 
   * This method is useful to change the initial solution provided
   * to a case.
   * <br>
   * {PRE: x is an instance of Case
   *       sol is an instance of CaseSolution
   * }<br>
   * {POST: desc is the new case solution of x.
   * }<br>
   * <br>
   * @param CaseSolution the new case solution. 
   */
    public void setCaseSolution(CaseSolution sol)
    {
       this.sol = sol;
    }  
    

  /**
   * Compares this case with a given one. <br>
   * The comparison is made among the case descriptions on a similarity
   * basis (SimilarityMeasure).
   * <br>
   * {PRE: x is an instance of Case,
   *       c2 is an instance of Case
   * }<br>
   * {POST: returns a similarity ratio among x and c2.
   * }<br>
   * @param c2 the other case to compare with. 
   * @return float the similatity ratio among this case and and c2 
   */
    public float similarTo(Case c2)
    {
       return this.getCaseDescription().similarTo(c2.getCaseDescription());
    }
    

  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of Case 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public String toString() //provisional
    {
       return ("\n############## Caso: " + getIdCase() + " #############" +
               "\n" + desc.toString() +
               "\n#########################################");
    }

}