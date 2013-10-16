package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.general.*;
import upc.lsi.kemlg.cbr.distances.*;

/**
 * This abstract class settles all the properties needed by any kind of Case Base Reasoner (CBR). <br>
 * <br>
 * In fact, all classes that implement any kind of CBR must inherit from
 * this class, and complete the abstract functions defined.<br><br>
 * This class and all its subclasses follow the java.util.Collection
 * interface, adding new features specifical of case libraries.<br>
 * <br>
 * The Case Base Reasoner is composed by:
 * <UL> <LI> a <b>Case Library</b> (Case Lib): a set of cases that represent the
 *           collected experience of a given domain (for instance, a set of
 *           past happenings and the solutions that were taken).
 *      <LI> the <b>Current Case</b>: the current problem to solve. Solving the
 *           current case means finding the past case (collected into the
 *           Case Lib that better fits to the current problem, and get the
 *           solution taken in the past.
 *      <LI> a <b>set of methods</b> to manage the Case Library and the Current Case.
 *           For instance, methods to retrieve the fittest past case, to adapt
 *           its solution to the current case, etc...    
 * </UL>
 * @see java.util.Collection
 * @see Case
 */
public abstract class CBR implements java.util.Collection, java.io.Serializable
{
  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================
  
  /** 
   * Object that assigns Case Identifiers (Case Ids) 
   */
   private IdGenerator genIds; 
   
  /** 
   *  Case Library Identifier (the name of the Case Library). 
   *   It will be a prefix of all the Case Ids.
   */
   private String nameCBR; 
   
  /** 
   *  Current Case the CBR will work with.
   */ 
   private Case currentCase;
   
  
  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  
  /**
   * Creates an empty CBR with a void name. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: true
   * }<br>
   * {POST: an instance of any non-abstract subclass of CBR has been created.
   * }<br>
   */
   public CBR()
   {
      genIds = new IdGenerator();
      nameCBR = "";
      currentCase = null;
   }
   
   
  /**
   * Creates an empty CBR with the given name. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: nameCBR is a string suitable (informative) for naming the Case Lib,
   * }<br>
   * {POST: an instance of any non-abstract subclass of CBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   */
   public CBR(String nameCBR)
   {
      genIds = new IdGenerator();
      this.nameCBR = nameCBR;
   }

   
  /**
   * Creates an empty CBR with a void name and sets genIds as the Case Identifiers' generator. <br> 
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br>
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: genIds is an Identifier generator.
   * }<br>
   * {POST: an instance of any non-abstract subclass of CBR has been created.
   * }<br>
   * @param genIds  the automatic Case Identifiers' generator that will be
   *                used.
   */
   public CBR(IdGenerator genIds)
   {
      this.genIds = genIds;
      nameCBR = "";
   }
     

  /**
   * Creates an empty CBR with the given name and sets genIds as the Case Identifiers' generator. <br>
   * (instead of a brand new generator with no ids assigned, as 
   *  default).<br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: nameCBR is a string suitable (informative) for naming the Case Lib,
   *       genIds is an Identifier generator.
   * }<br>
   * {POST: an instance of any non-abstract subclass of CBR has been created.
   * }<br>
   * <br>
   * @param nameCBR the name of the Case Library. 
   * @param genIds the automatic Case Identifiers' generator that will be
   *                used.
   */
   public CBR(String nameCBR, IdGenerator genIds)
   {
      this.genIds = genIds;
      this.nameCBR = nameCBR;
   }


  /**
   * Gets the Current Case the CBR methods are working with. <br> 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR
   *       the attribute table of x
   * }<br>
   * {POST: returns the Current Case x is working with.
   * }<br>
   * <br>
   * @return Case the Current Case, or null if it hasn't been set yet. 
   */
   public Case getCurrentCase()
   {
      return currentCase;
   }

         
  /**
   * Sets the Current Case the CBR methods will work with. <br> 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR and
   *       x has a non-void attribute table,
   *       o is a case instance, where all the attributes belong to 
   *       the attribute table of x
   * }<br>
   * {POST: o is the Current Case x will work with.
   * }<br>
   * <br>
   * @param o the Current Case 
   */
   public void setCurrentCase(Case o)
   {
      currentCase = o;
   }
    
    
  /**
   * Gets the name of the Case Library. <br> 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR
   *       the attribute table of x
   * }<br>
   * {POST: returns the name of the Case Library x.
   * }<br>
   * <br>
   * @return String the name of the Case Library. 
   */
   public String getNameCBR()
   {
      return nameCBR;
   }

         
  /**
   * Sets the name of the Case Library. <br> 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR
   *       n is a String with a valid name for x
   * }<br>
   * {POST: the name of Case Library x is n.
   * }<br>
   * <br>
   * @param n the name for the Case Library.
   */
   public void setNameCBR(String n)
   {
      nameCBR = n;
   }
    
  /**
   * Adds the specified element to this Case Lib. if it is not already present. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       o is any Java object
   * }<br>
   * {POST: returns true if o was added correctly to the case library (that is,
   *        if o is a case instance, where all the attributes belong to 
   *        the attribute table of x), 
   *        false if there was a problem to add it
   * }<br>
   * @param   o Case to be added to the Case Lib.
   * @return true if the Case Lib did not already contain the specified
   *          element.
   * @throws  ClassCastException if o is not a case or cannot be
   *          compared with the cases currently in the case library.
   */
   public boolean add (Object o) throws java.lang.ClassCastException 
   {
      boolean res = false;
       
      if (o instanceof Case)
      { 
         res = add((Case) o);
      }
      else
      {
         throw new java.lang.ClassCastException("Cannot add elem to Collection");
      }
      return res;
   }

   
  /** 
   * Returns true if this Case Lib. contains the specified element. <br>
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       o is any Java object
   * }<br>
   * {POST: returns true if o was found at the case library (that is,
   *        if o is a case instance, where all the attributes belong to 
   *        the attribute table of x and o was already inside the Case Lib), 
   *        false if there is not inside the Case Lib
   * }<br>
   * @param   o Case to be checked at the Case Lib.
   * @return true if the Case Lib contains the specified case.
   * @throws  ClassCastException if o is not a case or cannot be
   *          compared with the cases currently in the case library.
   */
  public boolean contains(Object o)
   {
      boolean res = false;
       
      if (o instanceof Case)
      { 
         res = contains((Case) o);
      }
      else
      {
         res=false;
      }
      return res;
   }


  /**     
   * Removes the given element from this Case Lib. if it is present.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       o is any Java object
   * }<br>
   * {POST: returns true if o was found at the case library and so, removed.
   *        false if there is not inside the Case Lib
   * }<br>
   * @param   o Case to be removed at the Case Lib.
   * @return true if the Case Lib contained the specified case.
   * @throws  ClassCastException if o is not a case.
   */
   public boolean remove(Object o) throws java.lang.ClassCastException
   {
      boolean res = false;
      
      if (o instanceof Case)
      { 
         res = remove((Case) o);
      }
      else
      {
         throw new java.lang.ClassCastException("Cannot remove this kind of Objects at the current Collection");       }
      return res;
   }
    

  
  //====||===============================================================
  //===\||/=============== metodos auxiliares ==========================
  //====\/===============================================================
  
  /**
   * Gets a new Case Identifier for a new case.
   * The identifier is composed by the Case Lib's name and a number. 
   * <br>
   * {PRE: true
   * }<br>
   * {POST: a new unique Case Identifier is given
   * }<br>
   * @return java.lang.String: the new Case Identifier. 
   */
   protected String assignId ()
   {
      return (nameCBR + "." + genIds.assignId());
   }
   
   
  
    
  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================
  

  /**
   * Adds the specified case to this Case Lib if it is not already present. <br>
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
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
   public abstract boolean add(Case o);
    

  /** 
   * Returns true if this Case Lib contains the specified case. <br>
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
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
   public abstract boolean contains(Case o);
    
    
  /**     
   * Removes the given case from this Case Lib if it is present.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns true if o was found at the case library and so, removed.
   *        false if o is not inside the Case Lib
   * }<br>
   * @param   o Case to be removed from the Case Lib.
   * @return true if the Case Lib contained the specified case.
   */
   public abstract boolean remove(Case o);


  /**
   * Adds all of the elements in the specified collection to this set.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
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
   public abstract boolean addAll(java.util.Collection c);

         
  /** 
   * Returns true if this Case Lib contains all of the elements in the specified collection. <br>
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if all the elements of c were found at the case library,
   *        false if there isn't any of the elements of c inside the Case Lib
   * }<br>
   * @param   c collection to be checked at the Case Lib.
   * @return true if the Case Lib contains all the elements in c.
   */
   public abstract boolean containsAll(java.util.Collection c);


  /**     
   * Removes the given elements from this Case Lib if they are present.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if this Case Lib changed as a result of the call
   * }<br>
   * @param   c collection of elements to be removed from the Case Lib.
   * @return true if the Case Lib changed.
   */
   public abstract boolean removeAll(java.util.Collection c);
   
   
  /**
   * Retains only the elements in this Case Lib that are contained in the specified collection. <br>
   * That is, removes from the Case Lib all of its elements that are not 
   * contained in the specified collection. 
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR,
   *       c is a Java collection of objects.
   * }<br>
   * {POST: returns true if this Case Lib changed as a result of the call
   * }<br>
   * @param   c collection of elements to be retained at the Case Lib, 
   *          removing the rest.
   * @return true if the Case Lib changed.
   */   
   public abstract boolean retainAll(java.util.Collection c);
    

  /**
   * Retrieves the case in the Case Lib most similar to the Current Case. <br>
   * <br>
   * Similarity among cases is computed by means of the Distamce Measure the
   * CBR has. 
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR
   * }<br>
   * {POST: returns the most similar case of the case lib and the
   *        similarity value among this case and the Current Case.
   * }<br>
   * @return RetrievedCase the most similar case of the case lib, with
   *          its associated similarity value to the Current Case.
   */   
   public abstract RetrievedCase retrieve1Case();
   
  /**
   * Retrieves the n cases in the Case Lib most similar to the Current Case. <br>
   * <br>
   * Similarity among cases is computed by means of the Distamce Measure the
   * CBR has. 
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR
   *       n is an integer value
   * }<br>
   * {POST: returns a list (a Java collection) of the m most similar cases of 
   *        the case lib (m less or equal n) and the similarity value among each
   *        case and the Current Case.
   * }<br>
   * @param   n the maximum number of cases we want as a result.
   * @return Collection the n most similar cases of the Case Lib, with
   *          their associated similarity value to the Current Case.
   */   
   public abstract TopNList retrieveNCases(int n);

    
  /**
   * Adapts the solution of the given case to set a solution for the Current Case. <br>
   * <br>
   * Usually the given case is a result from the retrieve methods. <br>
   * <br>
   * Adapting solutions from one case to another is a high domain-dependent task.
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR and
   *       x has a Current Case,
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns a case with the adapted solution of case o to the 
   *        Current Case.
   * }<br>
   * @return Case the case with the adapted solution.
   */   
   public abstract Case adapt1Case(Case o);
    
  /**
   * Adapts the solutions of the given collection of cases to set a solution
   * for the Current Case. <br>
   * <br>
   * Usually the given case collection is the result from the retrieveNCases
   * method. <br>
   * <br>
   * Adapting solutions from several cases to one is a high domain-dependent
   * task.
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR and
   *       x has a Current Case,
   *       c is a Java collection of objects, each one a case instance,
   *       where all the attributes belong to the attribute table of x
   * }<br>
   * {POST: returns a case with the adapted solution obtained by combining
   *        the solutions of the case collection c to the Current Case.
   * }<br>
   * @return Case the case with the adapted solution.
   */   
   public abstract Case adaptNCases(java.util.Collection c);

    
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
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of CBR and
   *       o is a case instance, where all the attributes belong to 
   *        the attribute table of x
   * }<br>
   * {POST: returns true if the case o has been "added" to the Case Lib, 
   *        completely or in part. 
   * }<br>
   * @return boolean the given Case has changed the state of the Case Lib.
   */   
   public abstract boolean learnCase(Case o);
   
}