package upc.lsi.kemlg.cbr;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;



/**
 * This abstract class defines a kind of class descriptions, where
 * each case is described a pair list (attribute,value), and where
 * similarity among case descriptions are computed as distances. <br>
 * Each pair (attribute,value) is implemented as an object of class
 * AttributeVal, which holds the name, qualitative and quantitative
 * values of the attribute.
 * <br>
 * The definitions of the accepted ranges and categories for each 
 * attribute are stored apart, in a attribute table (AttributeTable).
 * Each definition in that table is an instance of the AttributeDef
 * class.
 * <br>
 * As menctioned before, distances (class ComposedDistance) are used
 * as similarity measures (class SimilarityMeasure). So this class
 * follows the definition of the DistanceCollection interface, where
 * each element stored (in this case, an AttributeVal) should 
 * follow the DistanceComparable Interface.
 * <br>
 * Subclasses of this class define ways to structure the list of
 * attributes in memory.
 * @see CaseDescription
 * @see AttributeVal
 * @see AttributeDef
 * @see AttributeTable
 * @see SimilarityMeasure
 * @see ComposedDistance
 * @see DistanceCollection
 * @see DistanceComparable
 */
public abstract class AttributeValList extends CaseDescription 
                                       implements DistanceCollection, 
                                                  java.io.Serializable
{

  //====||===============================================================
  //===\||/=================== atributos ================================
  //====\/===============================================================

  /** The attribute table that contains the attribute definitions */
    private AttributeTable attrTab; //referencia a la tabla de atributos

  /** The distance measure to be applied to all the attributes*/
    private ComposedDistance dist;  
    


  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
  

  /**
   * Creates an empty AttributeValList, with the given attribute table and distance function. <br> 
   * <br>
   * <b>Note: this is an abstract class</b>, so this method is only useful
   * for any non-abstract subclasses.
   * {PRE: attrTab is instance of AttributeTable
   *       dist is an instance of ComposedDistance
   * }<br>
   * {POST: an instance of any non-abstract subclass of AttributeValList has been created.
   * }<br>
   */
    public AttributeValList(AttributeTable attrTab, ComposedDistance dist)
    {
       super((SimilarityMeasure) dist);
       this.attrTab = attrTab;
       this.dist = dist;
       
    }
   

  /**
   * Computes the distance among this AttributeValList and a given one. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       d2 is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns the computed distance among x and d2. 
   * }<br>
   * @param   d2 the AttributeValList to be considered.
   * @return float the distance among x and d2.
   */
    public float distanceTo(AttributeValList d2)
    {
       return (dist.computeDistance(this, d2));
    }   


  /**
   * Gets the attribute table associed to this AttributeValList. <br>
   * {PRE: x is an instance of any non-abstract subclass of AttributeValList.
   * }<br>
   * {POST: returns the attribute table associed to x.
   * }<br>
   * <br>
   * @return AttributeTable the attribute table associed to this AttributeValList. 
   */
    public AttributeTable getAttributeTable()
    {
        return attrTab;
    }
              

    //----- metodos abstractos de CaseDescription ----------
       
  /**
   * Compares this AttributeValList with a given one. <br>
   * The comparison is made on a similarity basis.
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of AttributeValList,
   *       c2 is an instance of any non-abstract subclass of CaseDescription
   * }<br>
   * {POST: if c2 is instance of AttributeValList, returns a similarity ratio
   *        among x and c2. Otherwise, an exception is thrown.
   * }<br>
   * @param c2 the other case description to compare with. 
   * @return float the similatity ratio among this AttributeValList and the
   *          case description c2.
   * @throws java.lang.ClassCastException The given case description is not an AttributeValList. 
   */
    public float similarTo(CaseDescription c2)
    {
       float res = -1;
       
       if (c2 instanceof AttributeValList)
       { 
           res = 1.0f - distanceTo((AttributeValList) c2);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot compare these two different kinds of case description");
       }
       return res;
    }

   
  /**
   * Gets the similarity measure. <br>
   * In this class, all similarity measures are distance functions. 
   * <br>
   * {PRE: x is an instance of any non-abstract subclass of AttributeValList.
   * }<br>
   * {POST: returns the similarity measure associed to x.
   * }<br>
   * <br>
   * @return SimilarityMeasure the similarity measure associed to this AttributeValList. 
   */
    public SimilarityMeasure getSimilarityMeasure()
    {
        return (SimilarityMeasure) dist;
    }
 

   //----- metodos abstractos de DistanceCollection ------
    
  /**
   * Computes the distance among this AttributeValList and a given DistanceCollection. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       d2 is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: if d2 is instance of AttributeValList, returns the computed distance
   *        among x and d2. Otherwise, an exception is thrown.
   * }<br>
   * @param   d2 the DistanceCollection to be considered.
   * @return float the distance among x and d2.
   * @throws java.lang.ClassCastException The given distance collection is not an AttributeValList. 
   */
    public float distanceTo(DistanceCollection d2)
    {
       float res =  -1;
      
       if (d2 instanceof AttributeValList)
       { 
           res = distanceTo((AttributeValList) d2);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot compare these two different kinds of Distance Collection");
       }
       return res;
    }   

    
  /**
   * Gets the current distance function that applies to all the attributes of this AttributeValList. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList
   * }<br>
   * {POST: returns the distance function that is applied for all the attribute values.
   * }<br>
   * @return ComposedDistance the distance function.
   */
    public ComposedDistance getCurrentDistance()
    {
       return dist;
    }
    

  /**
   * Adds the specified element to this AttributeValList, if it is not already present. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: returns true if o was added correctly to x,
   *        false if there was a problem to add it
   * }<br>
   * @param   o element to be added to the AttributeValList.
   * @return true if the AttributeValList is modified as a result
   *          of the invocation of this method.
   * @throws java.lang.ClassCastException The given element is not an AttributeVal. 
   */
    public boolean add (DistanceComparable o) throws java.lang.ClassCastException 
    {
       boolean res = false;
       
        if (o instanceof AttributeVal)
        { 
            res = add((AttributeVal) o);
        }
        else
        {
           throw new java.lang.ClassCastException("Cannot add elem to Collection");
        }
        return res;
    }
    
    
  /**
   * Adds all of the elements in the specified distance collection to this 
   * AttributeValList.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if all the elements in c were added correctly 
   *        to x, false if there were problems to add all of them.
   * }<br>
   * @param   c distance collection to be added to the AttributeValList.
   * @return true if the AttributeValList did not already contain any of the
   *          elements in c.
   * @throws java.lang.ClassCastException The given distance collection is not an AttributeValList. 
   */
    public boolean addAll(DistanceCollection c) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (c instanceof AttributeValList)
       { 
          res = addAll((AttributeValList) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot add this kind of Collection to the current Collection");
       }
       return res;
    }
    
      
  /** 
   * Returns true if this AttributeValList contains the specified element. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: if o is an instance of AttributeVal, returns true when o
   *        was found in x or false if not. Otherwise an exception is 
   *        thrown.
   * }<br>
   * @param   o element to be found in the AttributeValList.
   * @return true if the AttributeValList contains the specified element.
   * @throws java.lang.ClassCastException The given element is not an AttributeVal. 
   */
    public boolean contains(DistanceComparable o)
    {
       boolean res = false;
       
       if (o instanceof AttributeVal)
       { 
          res = contains((AttributeVal) o);
       }
       else
       {
          res=false;
       }
       return res;
    }
    
    
  /** 
   * Returns true if this AttributeValList contains all of the elements in the specified distance collection. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if all the elements of c were found at the AttributeValList,
   *        false if there isn't any of the elements of c inside the AttributeValList.
   *        Otherwise it throws an exception if c is not an instance of AttributeValList.
   *        
   * }<br>
   * @param   c distance collection to be checked at the AttributeValList.
   * @return true if the AttributeValList contains all the elements in c.
   * @throws java.lang.ClassCastException The given distance collection is not an AttributeValList. 
   */
    public boolean containsAll(DistanceCollection c)
    {
       boolean res = false;
       
       if (c instanceof AttributeValList)
       { 
          res = containsAll((AttributeValList) c);
       }
       else
       {
          res=false;
       }
       return res;
    }


  /**     
   * Removes the given element from this AttributeValList if it is present.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of DistanceComparable 
   * }<br>
   * {POST: returns true if o was found at the AttributeValList and so, removed.
   *        false if there isn't inside the AttributeValList.
   *        Otherwise it throws an exception if o is not an instance of AttributeVal.
   * }<br>
   * @param   o element to be removed at the AttributeValList.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given element is not an AttributeVal. 
   */
    public boolean remove(DistanceComparable o) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (o instanceof AttributeVal)
       { 
          res = remove((AttributeVal) o);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Objects at the current Collection");       }
       return res;
    }
    

  /**     
   * Removes the given elements from this AttributeValList if they are present.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c distance collection of elements to be removed from the AttributeValList.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given distance collection is not an AttributeValList. 
   */
    public boolean removeAll(DistanceCollection c) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (c instanceof AttributeValList)
       { 
          res = removeAll((AttributeValList) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Collection to the current Collection");       
       }
       return res;
    }
    

  /**
   * Retains only the elements in this AttributeValList that are contained in the specified distance collection. <br>
   * That is, removes from the AttributeValList all of its elements that are not 
   * contained in the specified collection. 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of DistanceCollection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c collection of elements to be retained at the AttributeValList, 
   *          removing the rest.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given distance collection is not an AttributeValList. 
   */   
    public boolean retainAll(DistanceCollection c) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (c instanceof AttributeValList)
       { 
          res = retainAll((AttributeValList) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Collection to the current Collection");
       }
       return res;
    }
  
    
    //--------- métodos abstractos de Collection ------------

  /**
   * Adds the specified element to this AttributeValList, if it is not already present. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a subclass of Object 
   * }<br>
   * {POST: returns true if o was added correctly to x,
   *        false if there was a problem to add it
   * }<br>
   * @param   o element to be added to the AttributeValList.
   * @return true if the AttributeValList is modified as a result
   *          of the invocation of this method.
   * @throws java.lang.ClassCastException The given element is not an instance of DistanceComparable. 
   */
    public boolean add (Object o) throws java.lang.ClassCastException 
    {
       boolean res = false;
       
        if (o instanceof DistanceComparable)
        { 
            res = add((DistanceComparable) o);
        }
        else
        {
           throw new java.lang.ClassCastException("Cannot add elem to Collection");
        }
        return res;
    }
    

  /**
   * Adds all of the elements in the specified collection to this 
   * AttributeValList.<br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of Collection 
   * }<br>
   * {POST: returns true if all the elements in c were added correctly 
   *        to x, false if there were problems to add all of them.
   * }<br>
   * @param   c distance collection to be added to the AttributeValList.
   * @return true if the AttributeValList did not already contain any of the
   *          elements in c.
   * @throws java.lang.ClassCastException The given collection is not an DistanceCollection. 
   */
    public boolean addAll(java.util.Collection c) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (c instanceof DistanceCollection)
       { 
          res = addAll((DistanceCollection) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot add this kind of Collection to the current Collection");
       }
       return res;
    }
      

  /** 
   * Returns true if this AttributeValList contains the specified element. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of Object 
   * }<br>
   * {POST: if o is an instance of DistanceComparable, returns true when o
   *        was found in x or false if not. Otherwise an exception is 
   *        thrown.
   * }<br>
   * @param   o element to be found in the AttributeValList.
   * @return true if the AttributeValList contains the specified element.
   * @throws java.lang.ClassCastException The given element is not an instance of DistanceComparable. 
   */
    public boolean contains(Object o)
    {
       boolean res = false;
       
       if (o instanceof DistanceComparable)
       { 
          res = contains((DistanceComparable) o);
       }
       else
       {
          res=false;
       }
       return res;
    }
    
  /** 
   * Returns true if this AttributeValList contains all of the elements in the specified collection. <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of Collection 
   * }<br>
   * {POST: returns true if all the elements of c were found at the AttributeValList,
   *        false if there isn't any of the elements of c inside the AttributeValList.
   *        Otherwise it throws an exception if c is not an instance of AttributeValList.
   *        
   * }<br>
   * @param   c distance collection to be checked at the AttributeValList.
   * @return true if the AttributeValList contains all the elements in c.
   * @throws java.lang.ClassCastException The given collection is not an DistanceCollection. 
   */
    public boolean containsAll(java.util.Collection c)
    {
       boolean res = false;
       
       if (c instanceof DistanceCollection)
       { 
          res = containsAll((DistanceCollection) c);
       }
       else
       {
          res=false;
       }
       return res;
    }

  /**     
   * Removes the given element from this AttributeValList if it is present.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of Object 
   * }<br>
   * {POST: returns true if o was found at the AttributeValList and so, removed.
   *        false if there isn't inside the AttributeValList.
   *        Otherwise it throws an exception if o is not an instance of DistanceComparable.
   * }<br>
   * @param   o element to be removed at the AttributeValList.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given element is not an instance of DistanceComparable. 
   */
    public boolean remove(Object o) throws java.lang.ClassCastException

    {
       boolean res = false;
       
       if (o instanceof DistanceComparable)
       { 
          res = remove((DistanceComparable) o);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Objects at the current Collection");       }
       return res;
    }
    
  /**     
   * Removes the given elements from this AttributeValList if they are present.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of Collection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c collection of elements to be removed from the AttributeValList.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given collection is not an DistanceCollection. 
   */
    public boolean removeAll(java.util.Collection c) throws java.lang.ClassCastException

    {
       boolean res = false;
       
       if (c instanceof DistanceCollection)
       { 
          res = removeAll((DistanceCollection) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Collection to the current Collection");       }
       return res;
    }
    
  /**
   * Retains only the elements in this AttributeValList that are contained in the specified distance collection. <br>
   * That is, removes from the AttributeValList all of its elements that are not 
   * contained in the specified collection. 
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of Collection 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c collection of elements to be retained at the AttributeValList, 
   *          removing the rest.
   * @return true if the AttributeValList changed as a result of the call.
   * @throws java.lang.ClassCastException The given collection is not an DistanceCollection. 
   */   
    public boolean retainAll(java.util.Collection c) throws java.lang.ClassCastException
    {
       boolean res = false;
       
       if (c instanceof DistanceCollection)
       { 
          res = retainAll((DistanceCollection) c);
       }
       else
       {
          throw new java.lang.ClassCastException("Cannot remove this kind of Collection to the current Collection");       }
       return res;
    }
  
 

  //====||===============================================================
  //===\||/====== metodos a implementar en las subclases ================
  //====\/===============================================================


  /**
   * Adds the specified attribute to this AttributeValList, if it is not already present. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was added correctly to x,
   *        false if there was a problem to add it
   * }<br>
   * @param   o attribute to be added to the v.
   * @return true if the AttributeValList is modified as a result
   *          of the invocation of this method.
   */
    public abstract boolean add(AttributeVal o);
    
 
  /**
   * Adds all of the attributes in the specified AttributeValList to this 
   * AttributeValList.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if all the attributes in c were added correctly 
   *        to x, false if there were problems to add all of them.
   * }<br>
   * @param   c AttributeValList to be added to the AttributeValList.
   * @return true if the AttributeValList did not already contain any of the
   *          attributes in c.
   */
    public abstract boolean addAll(AttributeValList c);
    
 
  /** 
   * Returns true if this AttributeValList contains the specified attribute. <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was found in x, 
   *        false otherwise
   * }<br>
   * @param   o attribute to be found in the AttributeValList.
   * @return true if the AttributeValList contains the specified attribute.
   */
    public abstract boolean contains(AttributeVal o);

 
  /** 
   * Returns true if this AttributeValList contains all of the attributes in the specified AttributeValList. <br>
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if all the attributes of c were found at the AttributeValList,
   *        false if there isn't any of the attributes of c inside the AttributeValList
   * }<br>
   * @param   c collection to be checked at the AttributeValList.
   * @return true if the AttributeValList contains all the attributes in c.
   */
    public abstract boolean containsAll(AttributeValList c);
    
 
  /**     
   * Removes the given attribute from this AttributeValList if it is present.
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * <br>
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       o is an instance of a non-abstract subclass of AttributeVal 
   * }<br>
   * {POST: returns true if o was found at the AttributeValList and so, removed.
   *        false if there isn't inside the AttributeValList.
   * }<br>
   * @param   o attribute to be removed at the AttributeValList.
   * @return true if the AttributeValList contained the specified attribute.
   */
    public abstract boolean remove(AttributeVal o);

 
  /**     
   * Removes the given attributes from this AttributeValList if they are present.
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c AttributeValList to be removed from the AttributeValList.
   * @return true if the AttributeValList changed as a result of the call.
   */
    public abstract boolean removeAll(AttributeValList c);
    
 
  /**
   * Retains only the attributes in this AttributeValList that are contained in the specified AttributeValList. <br>
   * That is, removes from the AttributeValList all of its attributes that are not 
   * contained in the specified AttributeValList. 
   * <br>
   * <b> Note: this is an abstract method.</b> It should be defined in the
   * subclasses.
   * {PRE: x is an instance of a non-abstract subclass of AttributeValList,
   *       c is an instance of a non-abstract subclass of AttributeValList 
   * }<br>
   * {POST: returns true if x changed as a result of the call
   * }<br>
   * @param   c AttributeValList to be retained at the AttributeValList, 
   *          removing the rest.
   * @return true if the AttributeValList changed as a result of the call.
   */   
    public abstract boolean retainAll(AttributeValList c);


  //---------- metodo abstracto de CaseDescription ------------ 
  /**
   * Returns a string representation of the object. <br>
   * {PRE: x is an instance of any non-abstract subclass of AttributeValList. 
   * }<br>
   * {POST: returns a string representation of x. 
   * }<br>
   * @return String a string representation of the object.    
   */
    public abstract String toString();

}