package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.general.*;

/**
 * This class defines a Case Base Reasoner (CBR) with a plain structure 
 * of its Case Library (Case Lib).<br>
 * <br>
 * As it implements one kind of CBR, it inherits from the CBR class, 
 * and puts code to all the abstract inherited methods.<br>
 * <br>
 * <b>Note: 
 * <ul><li> the adaptation step, either from one case or from N cases,
 *          is a high domain-dependent task. This class implements the
 *          default adaptation to be made, that is, <i>no adaptation</i>.
 *          it always returns a case as it is in the Case Library. Is a 
 *          task of the subclasses to override the adaptation methods 
 *          to put the proper adaptation to the given domain.</b>
 *     </li>
 *  <b><li> the learning step implemented is the most simple one and it
 *          is the default one: to learn the case anytime the method is 
 *          called, adding it to the Case Library with no filtering or
 *          reorganization of the library. Is a task of the subclasses 
 *          to override the learning method to fit it to the particular
 *          needs.
 *     </li>
 * </ul></b>
 * @see java.util.Collection
 * @see CBR
 */
public class PlainCBR extends CBR implements java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================
  
  /**
   * The Collection that stores the Case Library
   */
   private java.util.Collection caseLib; 
   


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  
  /**
   * Creates an empty PlainCBR with a void name. <br> 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: an instance of PlainCBR has been created.
   * }<br>
   */
   public PlainCBR()
   {
      super();
      caseLib = new java.util.Vector();
   }
   

  /**
   * Creates an empty PlainCBR with the given name. <br> 
   * <br>
   * {PRE: nameCBR is a string suitable (informative) for naming the 
   *       Case Library,
   * }<br>
   * {POST: an instance of PlainCBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   */
   public PlainCBR(String nameCBR)
   {
      super(nameCBR);
      caseLib = new java.util.Vector();
   }
   

  /**
   * Creates an empty PlainCBR with a void name and sets genIds as the Case Identifiers' generator. <br> 
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br>
   * <br>
   * {PRE: genIds is an instance of IdGenerator.
   * }<br>
   * {POST: an instance of PlainCBR has been created.
   * }<br>
   * @param genIds  the automatic Case Identifiers' generator that will be
   *                used.
   */
   public PlainCBR(IdGenerator genIds)
   {
      super(genIds);
      caseLib = new java.util.Vector();
   }
     

  /**
   * Creates an empty PlainCBR with the given name and sets genIds as the Case Identifiers' generator. <br>
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br> 
   * <br>
   * {PRE: nameCBR is a string suitable (informative) for naming the Case Lib,
   *       genIds is an Identifier generator.
   * }<br>
   * {POST: an instance of PlainCBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   * @param genIds the automatic Case Identifiers' generator that will be
   *                used.
   */
   public PlainCBR(String nameCBR, IdGenerator genIds)
   {
      super(nameCBR,genIds);
      caseLib = new java.util.Vector();
   }
     
   
   //Implementación de los métodos abstractos de CBR
    
  /**
   * Adds the specified case to this Case Lib if it is not already present. <br>
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       o is a case instance, where all the attributes belong to 
   *       the attribute table of x
   * }<br>
   * {POST: returns true if o was added correctly to the case library, false
   *        if there was a problem to add it
   * }<br>
   * @param   o Case to be added to the Case Lib.
   * @return true if the Case Lib did not already contain the specified
   *          element.
   * @throws  ClassCastException if the specified object cannot be
   *          compared with the cases currently in the case library.
   */
   public boolean add(Case o)
   {
      o.setIdCase(assignId());
      return caseLib.add(o);
   }
   
    
  /** 
   * Returns true if this Case Lib contains the specified case. <br>
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns true if o was found at the case library,
   *        false if there isn't inside the Case Lib
   * }<br>
   * @param   o Case to be checked at the Case Lib.
   * @return true if the Case Lib contains the specified case.
   * @throws  ClassCastException if o cannot be compared with
   *          the cases currently in the case library.
   */
   public boolean contains(Case o)
   {
      return caseLib.contains(o);
   }
    
    
  /**     
   * Removes the given case from this Case Lib if it is present.
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns true if o was found at the case library and so, removed.
   *        false if o is not inside the Case Lib
   * }<br>
   * @param   o Case to be removed from the Case Lib.
   * @return true if the Case Lib contained the specified case.
   */
   public boolean remove(Case o)
   {
      return caseLib.remove(o);
   } 


  /**
   * Adds all of the elements in the specified collection to this set.
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if all the elements in c were added correctly 
   *        to the case library, false if there were problems to add all
   *        them.
   * }<br>
   * @param   c collection to be added to the Case Lib.
   * @return true if the Case Lib did not already contain any of the
   *          elements in c.
   */
   public boolean addAll(java.util.Collection c)
   {
      return caseLib.addAll(c);
   }


  /** 
   * Returns true if this Case Lib contains all of the elements in the specified collection. <br>
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if all the elements of c were found at the case library,
   *        false if there isn't any of the elements of c inside the Case Lib
   * }<br>
   * @param   c collection to be checked at the Case Lib.
   * @return true if the Case Lib contains all the elements in c.
   */
   public boolean containsAll(java.util.Collection c)
   {
      return caseLib.containsAll(c);
   }


  /**     
   * Removes the given elements from this Case Lib if they are present.
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if this Case Lib changed as a result of the call
   * }<br>
   * @param   c collection of elements to be removed from the Case Lib.
   * @return true if the Case Lib changed.
   */
   public boolean removeAll(java.util.Collection c)
   {
      return caseLib.removeAll(c);
   }

    
  /**
   * Retains only the elements in this Case Lib that are contained in the specified collection. <br>
   * That is, removes from the Case Lib all of its elements that are not 
   * contained in the specified collection. 
   * <br>
   * {PRE: x is an instance of PlainCBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if this Case Lib changed as a result of the call
   * }<br>
   * @param   c collection of elements to be retained at the Case Lib, 
   *          removing the rest.
   * @return true if the Case Lib changed.
   */   
   public boolean retainAll(java.util.Collection c)
   {
      return caseLib.retainAll(c);
   }

     
  /**
   * Retrieves the case in the Case Lib most similar to the Current Case. <br>
   * <br>
   * Similarity among cases is computed by means of the Distamce Measure the
   * CBR has. 
   * {PRE: x is an instance of PlainCBR
   * }<br>
   * {POST: returns the most similar case of the case lib and the
   *        similarity value among this case and the Current Case.
   * }<br>
   * @return RetrievedCase the most similar case of the case lib, with
   *          its associated similarity value to the Current Case.
   */   
   public RetrievedCase retrieve1Case()
   {
      return (RetrievedCase) retrieveNCases(1).getFirst();
   }
   
   
  /**
   * Retrieves the n cases in the Case Lib most similar to the Current Case. <br>
   * <br>
   * Similarity among cases is computed by means of the Distamce Measure the
   * CBR has. 
   * {PRE: x is an instance of PlainCBR
   *       n is an integer value
   * }<br>
   * {POST: returns a list (a Java collection) of the m most similar cases of 
   *        the case lib (m less or equal n) and the similarity value among each
   *        case and the Current Case.
   * }<br>
   * @param   n the maximum number of cases we want as a result.
   * @return Collection the n most similar cases of the Case Lib, with
   *          their associated similarity value the Current Case.
   */    
   public TopNList retrieveNCases(int n)
   {
       TopNList retrieveList;
       java.util.Iterator i;
       Case currentC, auxC;
       RetrievedCase retrC;
       float auxD;
       
       retrieveList = new TopNList();
       
       currentC = getCurrentCase();
       
       for (i=caseLib.iterator(); i.hasNext();)
       {
          auxC = (Case) i.next();
          auxD = currentC.similarTo(auxC);
          
          retrC = new RetrievedCase(auxC, auxD);
          retrieveList.add(retrC);
       }   
       
       return retrieveList;
   }  
    
    
  /**
   * Adapts the solution of the given case to set a solution for the Current Case. <br>
   * <br>
   * Usually the given case is a result from the retrieve methods. <br>
   * <br>
   * Adapting solutions from one case to another is a high domain-dependent task.
   * <b> Note: this class implements the default adaptation to be made, that is,
   * <i>no adaptation at all</i></b>. It gives back the case as it is. Is a task of the
   * subclasses to override this method to put the proper adaptation to the
   * given domain.
   * {PRE: x is an instance of PlainCBR and
   *       x has a Current Case,
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns a case with the adapted solution of case o to the 
   *        Current Case.
   * }<br>
   * @return Case the case with the adapted solution.
   */   
   public Case adapt1Case(Case o)
   {
      return o;
   }
    
    
  /**
   * Adapts the solutions of the given collection of cases to set a solution
   * for the Current Case. <br>
   * <br>
   * Usually the given case collection is the result from the retrieveNCases
   * method. <br>
   * <br>
   * Adapting solutions from several cases to one is a high domain-dependent
   * task.
   * <b> Note: this class implements the default adaptation to be made, that is,
   * no adaptation at all.</b> It gives back the best case of the list, as it is. 
   * Is a task of the subclasses to override this method to put the proper 
   * adaptation to the given domain.
   * {PRE: x is an instance of PlainCBR and
   *       x has a Current Case,
   *       c is a Java collection of objects, each one a case instance,
   *       where all the attributes belong to the attribute table of x
   * }<br>
   * {POST: returns a case with the adapted solution obtained by combining
   *        the solutions of the case collection c to the Current Case.
   * }<br>
   * @return Case the case with the adapted solution.
   */   
   public Case adaptNCases(java.util.Collection c)
   {
      return getBestCase(c).getCase();
   }

    
  /**
   * Learns the given case, adding it at the Case Library. <br>
   * <br>
   * Learning means adding the experience of this case to the past experience,
   * that is, the Case Lib. But:
   * <UL> <LI> adding does not always mean to add the whole case to the 
   *           Case Lib... sometimes the new case and some previous cases are
   *           "joint" in a case that describes them all... These join 
   *           decisions depend on the domain most of times.
   *      <LI> the decision of learning the case or not 
   *           is usually taken analizing how informative it is.... 
   *           a decision quite domain-dependent. <br>
   * <br>
   * <b> Note:the learning step implemented </b> is the most simple one 
   *     and it is the default one: to learn the case anytime the method is 
   *     called, adding it to the Case Library with no filtering or
   *     reorganization of the library. Is a task of the subclasses 
   *     to override the learning method to fit it to the particular
   *     needs.
   * <br>
   * {PRE: x is an instance of PlainCBR and
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns true if the case o has been "added" to the Case Lib, 
   *        completely or in part. 
   * }<br>
   * @return boolean the given Case has changed the state of the Case Lib.
   */   
   public boolean learnCase(Case o)
   {
      return add(o);
   }  
   

    //Implementación de los métodos abstractos de Collection que no 
    //implementa CBR

  /**
   * Returns an iterator over the elements (cases) in this PlainCBR. <br>
   * The order of cases returned is not guaranted.
   * <br>
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: returns a java iterator to browse through the elements.
   * }<br>
   * @return Iterator a Java iterator for this structure
   */
   public java.util.Iterator iterator()
   {
      return caseLib.iterator();
   }

    
  /**
   * Returns the size of the PlainCBR structure. <br>
   * The size of the PlainCbr is the number of cases stored.
   * If the Case Library contains more than
   * Integer.MAX_VALUE elements, returns Integer.MAX_VALUE.
   * <br>
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: returns the number of cases stored in x.
   * }<br>
   * @return int the number of cases in the Case Library.
   */
   public int size()
   {
      return caseLib.size();
   }

    
  /**
   * Returns true if the PlainCBR structure contains no elements. <br>
   * <br>
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: returns true if the the number of cases stored in x is 0.
   * }<br>
   * @return boolean true if there are no cases in the Case Library.
   */
   public boolean isEmpty()
   {
      return caseLib.isEmpty();
   }

    
  /**
   * Returns an array containing all of the elements in this collection. <br>
   * This method does not guarantees the order of the elements returned.
   * <br>
   * The returned array will be "safe" in that no references to it are
   * maintained by this collection. (In other words, this method must
   * allocate a new array even if this collection is backed by an array).
   * The caller is thus free to modify the returned array.
   * <br>
   * This method acts as bridge between array-based and collection-based
   * APIs.
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: returns an array of objects containing a copy of all the 
   *        elements (cases) in x.
   * }<br>
   * @return Object[] an array containing all the elements.
   */
   public Object[] toArray()
   {
      return caseLib.toArray();
   }

    
  /**
   * Returns an array containing all of the elements in this collection
   * whose runtime type is that of the specified array. If the collection
   * fits in the specified array, it is returned therein. Otherwise, a new
   * array is allocated with the runtime type of the specified array and
   * the size of this collection.
   * <br>
   * If this collection fits in the specified array with room to spare 
   * (i.e., the array has more elements than this collection), the element
   * in the array immediately following the end of the collection is set
   * to null. This is useful in determining the length of this collection
   * only if the caller knows that this collection does not contain any null
   * elements.)   
   * This method does not guarantees the order of the elements returned.
   * <br>
   * Like the toArray method, this method acts as bridge between 
   * array-based and collection-based APIs. Further, this method allows
   * precise control over the runtime type of the output array, and may,
   * under certain circumstances, be used to save allocation costs.
   * <br>
   * Note that toArray(new Object[0]) is identical in function to toArray().   
   * {PRE: x is an instance of PlainCBR and
   *       a is an empty array of objects 
   * }<br>
   * {POST: returns an array of all elements (cases) in x which runtime
   *        type is the same of array a.
   * }<br>
   * @return Object[] an array containing some of the elements.
   */
   public Object[] toArray(Object[] a)
   {
      return caseLib.toArray(a);
   }  

  
  /**
   * Removes all of the elements from this collection. <br>
   * This collection will be empty after the call to the method.
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: x is empty. 
   * }<br>
   */  
   public synchronized void clear()
   {
      caseLib.clear();
   }


  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of PlainCBR 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
   public String toString() //provisional
   {
      return ("\nCBR: numero de casos = " + caseLib.size() + "\n" + caseLib.toString());
   }


  //====||===============================================================
  //===\||/=============== metodos auxiliares ==========================
  //====\/===============================================================
  
  /**
   * Returns the best case of a Collection.
   * {PRE: c is a Collection which elements are all instances of
   *         class Case or any of its subclasses, and c is ordered
   *         from the most similar case to the least one.
   * }<br>
   * {POST: returns the first element of c, that is, the best one.
   * }<br>
   * @return RetrievedCase the best case of the Collection.
   * @throws ClassCastException if the Collection c is not a 
   *         java.util.SortedSet or a upc.lsi.kemlg.general.TopNList  
   */
   private RetrievedCase getBestCase(java.util.Collection c)
   {
      RetrievedCase aux=null;
     
      //Por razones de optimización en tiempo, este método solo
      // funciona cuando la Collection que se pasa esta ordenada,
      // ya sea una collection ordenada por Java (SortedSet) como
      // una TopNList, una collection especifica del KEMLG que
      // esta también ordenada.
      if (c instanceof java.util.SortedSet)
      {
         aux = (RetrievedCase)((java.util.SortedSet) c).first();
      }
      else if (c instanceof TopNList)
      {
         aux = (RetrievedCase)((TopNList) c).getFirst();
      }
      else
      {
         throw new java.lang.ClassCastException("Cannot find best case in this type of collection");
      } 
      return aux;
   }
      
}